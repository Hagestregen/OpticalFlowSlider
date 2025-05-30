import sys
import tty
import termios
from dynamixel_sdk import PortHandler, PacketHandler

class DynamixelMXController:
    # Control Table Addresses
    ADDR_TORQUE_ENABLE = 64
    ADDR_GOAL_POSITION = 116
    ADDR_PRESENT_POSITION = 132
    ADDR_IS_MOVING = 123
    ADDR_MIN_POSITION_LIMIT = 52
    ADDR_MAX_POSITION_LIMIT = 48
    ADDR_PRESENT_VELOCITY = 128
    ADDR_Profile_Acceleration = 108
    ADDR_Profile_Velocity = 112
    ADDR_OPERATING_MODE = 11

    # Default Settings
    PROTOCOL_VERSION = 2.0
    BAUDRATE = 1000000
    TORQUE_ENABLE = 1
    TORQUE_DISABLE = 0
    MOVING_STATUS_THRESHOLD = 20
    DXL_MINIMUM_POSITION_VALUE = 1000
    DXL_MAXIMUM_POSITION_VALUE = 2000
    DXL_MINIMUM_LIMIT_VALUE = 0
    DXL_MAXIMUM_LIMIT_VALUE = 2500
    PROFILE_ACCELERATION = 20
    PROFILE_VELOCITY = 100


    def __init__(self, device_name='/dev/ttyUSB0', motor_id=1, goal_positions=None):
        self.device_name = device_name
        self.motor_id = motor_id
        self.min_limit = self.DXL_MINIMUM_LIMIT_VALUE
        self.max_limit = self.DXL_MAXIMUM_LIMIT_VALUE
        self.port_handler = PortHandler(self.device_name)
        self.packet_handler = PacketHandler(self.PROTOCOL_VERSION)
        self.goal_positions = goal_positions or [
            self.DXL_MINIMUM_POSITION_VALUE,
            self.DXL_MAXIMUM_POSITION_VALUE,
        ]
        self.current_goal_index = 0

    #Set the acceleration and velocity limits
    
    def set_vel_and_accel(self, vel=100, accel=20):
        self.PROFILE_VELOCITY = vel
        self.PROFILE_ACCELERATION = accel
        result, error = self.packet_handler.write4ByteTxRx(
            self.port_handler, self.motor_id, self.ADDR_Profile_Acceleration, self.PROFILE_ACCELERATION
        )
        self.check_comm_result(result, error, "Set Profile Acceleration")
        result, error = self.packet_handler.write4ByteTxRx(
            self.port_handler, self.motor_id, self.ADDR_Profile_Velocity, self.PROFILE_VELOCITY
        )
        self.check_comm_result(result, error, "Set Profile Velocity")

    #Set the minimum and maximum position limits
    def set_position_limits(self):
        result, error = self.packet_handler.write4ByteTxRx(
            self.port_handler, self.motor_id, self.ADDR_MIN_POSITION_LIMIT, self.DXL_MINIMUM_LIMIT_VALUE
        )
        self.check_comm_result(result, error, "Set min position limit")
        result, error = self.packet_handler.write4ByteTxRx(
            self.port_handler, self.motor_id, self.ADDR_MAX_POSITION_LIMIT, self.DXL_MAXIMUM_LIMIT_VALUE
        )
        self.check_comm_result(result, error, "Set max position limit")

    def open_port(self):
        if not self.port_handler.openPort():
            raise Exception("Failed to open the port")
        print("Succeeded to open the port")

        if not self.port_handler.setBaudRate(self.BAUDRATE):
            raise Exception("Failed to set the baudrate")
        print("Succeeded to change the baudrate")

    def enable_torque(self):
        result, error = self.packet_handler.write1ByteTxRx(
            self.port_handler, self.motor_id, self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE
        )
        self.check_comm_result(result, error, "Torque enable")
        print("Torque enabled")

    def disable_torque(self):
        result, error = self.packet_handler.write1ByteTxRx(
            self.port_handler, self.motor_id, self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE
        )
        self.check_comm_result(result, error, "Torque disable")
        print("Torque disabled")

    def set_goal_position(self, position):
        result, error = self.packet_handler.write4ByteTxRx(
            self.port_handler, self.motor_id, self.ADDR_GOAL_POSITION, position
        )
        self.check_comm_result(result, error, "Set goal position")
        # print(f"[ID:{self.motor_id}] Set goal position: {position}")
        
    def get_goal_position(self):
        goal_position, result, error = self.packet_handler.read4ByteTxRx(
            self.port_handler, self.motor_id, self.ADDR_GOAL_POSITION
        )
        self.check_comm_result(result, error, "Read goal position")
        return goal_position

    def get_present_position(self):
        position, result, error = self.packet_handler.read4ByteTxRx(
            self.port_handler, self.motor_id, self.ADDR_PRESENT_POSITION
        )
        self.check_comm_result(result, error, "Read present position")
        return position
    
    def get_present_velocity(self):
        velocity, result, error = self.packet_handler.read4ByteTxRx(
            self.port_handler, self.motor_id, self.ADDR_PRESENT_VELOCITY
        )
        self.check_comm_result(result, error, "Read present velocity")
        # Convert the raw unsigned value to a signed integer if necessary
        if velocity > 0x7fffffff:
            velocity = velocity - 4294967296
        # print(f"Present velocity: {velocity}")
        return velocity


    def is_moving(self):
        is_moving, result, error = self.packet_handler.read1ByteTxRx(
            self.port_handler, self.motor_id, self.ADDR_IS_MOVING
        )
        self.check_comm_result(result, error, "Check is_moving")
        return is_moving

    def move_to_goal(self):
        goal_position = self.goal_positions[self.current_goal_index]
        self.set_goal_position(goal_position)

        while self.is_moving():
            present_position = self.get_present_position()
            print(f"[ID:{self.motor_id}] GoalPos:{goal_position}  PresPos:{present_position}")

        self.current_goal_index = 1 - self.current_goal_index  # Toggle between 0 and 1

    def close_port(self):
        self.port_handler.closePort()
        print("Port closed")

    def check_comm_result(self, result, error, action):
        if result != 0:
            print(f"Communication error in {action}: {self.packet_handler.getTxRxResult(result)}")
        elif error != 0:
            print(f"Packet error in {action}: {self.packet_handler.getRxPacketError(error)}")

    @staticmethod
    def getch():
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            return sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


if __name__ == "__main__":
    print("This script defines motor control logic and is meant to be imported by the ROS 2 script.")
