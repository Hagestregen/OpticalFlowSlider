import sys
import tty
import termios
from dynamixel_sdk import PortHandler, PacketHandler

class DynamixelMXController:
    # Control Table Addresses
    ADDR_TORQUE_ENABLE = 64
    ADDR_GOAL_POSITION = 116
    ADDR_PRESENT_POSITION = 132

    # Default Settings
    PROTOCOL_VERSION = 2.0
    BAUDRATE = 1000000
    TORQUE_ENABLE = 1
    TORQUE_DISABLE = 0
    MOVING_STATUS_THRESHOLD = 20
    DXL_MINIMUM_POSITION_VALUE = 1500
    DXL_MAXIMUM_POSITION_VALUE = 3000

    def __init__(self, device_name='/dev/ttyUSB0', motor_id=1):
        self.device_name = device_name
        self.motor_id = motor_id
        self.port_handler = PortHandler(self.device_name)
        self.packet_handler = PacketHandler(self.PROTOCOL_VERSION)
        self.goal_positions = [
            self.DXL_MINIMUM_POSITION_VALUE,
            self.DXL_MAXIMUM_POSITION_VALUE,
        ]
        self.current_goal_index = 0

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

    def disable_torque(self):
        result, error = self.packet_handler.write1ByteTxRx(
            self.port_handler, self.motor_id, self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE
        )
        self.check_comm_result(result, error, "Torque disable")

    def set_goal_position(self, position):
        result, error = self.packet_handler.write4ByteTxRx(
            self.port_handler, self.motor_id, self.ADDR_GOAL_POSITION, position
        )
        self.check_comm_result(result, error, "Set goal position")

    def get_present_position(self):
        position, result, error = self.packet_handler.read4ByteTxRx(
            self.port_handler, self.motor_id, self.ADDR_PRESENT_POSITION
        )
        self.check_comm_result(result, error, "Read present position")
        return position

    def move_to_goal(self):
        goal_position = self.goal_positions[self.current_goal_index]
        self.set_goal_position(goal_position)

        while True:
            present_position = self.get_present_position()
            print(f"[ID:{self.motor_id}] GoalPos:{goal_position}  PresPos:{present_position}")

            if abs(goal_position - present_position) <= self.MOVING_STATUS_THRESHOLD:
                break

        self.current_goal_index = 1 - self.current_goal_index  # Toggle between 0 and 1

    def close_port(self):
        self.port_handler.closePort()

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


def main():
    controller = DynamixelMXController()
    try:
        controller.open_port()
        controller.enable_torque()

        while True:
            print("Press any key to continue! (or press ESC to quit!)")
            if controller.getch() == chr(0x1b):  # ESC key
                break

            controller.move_to_goal()

    finally:
        controller.disable_torque()
        controller.close_port()


if __name__ == "__main__":
    main()
