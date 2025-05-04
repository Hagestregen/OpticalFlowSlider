import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from dynamixel_sdk import PortHandler, PacketHandler
import utils

class MotorPublisherNode(Node):
    def __init__(self, controller):
        super().__init__('motor_publisher')
        self.controller = controller

        # Publisher for present velocity
        self.velocity_pub = self.create_publisher(
            Float64,
            '/motor/present_velocity',
            10
        )

        # Subscriber to PID control input
        self.control_sub = self.create_subscription(
            Float64,
            '/motor/control_input',
            self.control_callback,
            10
        )

        # Timer to publish velocity feedback
        self.timer = self.create_timer(0.001, self.publish_velocity_callback)

    def control_callback(self, msg):
        # Convert linear velocity (m/s) to RPM
        linear_velocity = msg.data
        # self.get_logger().info(f"MotorNode: Recieved {linear_velocity}")
        # rpm = utils.linear_velocity_mps_to_rpm(linear_velocity)
        # raw_velocity = utils.rpm_to_raw(rpm)
        dxl_cmd = utils.linear_velocity_to_dynamixel_input(linear_velocity)
        velocity_limit = 100  # Replace with self.controller.get_velocity_limit() after adding
        # raw_velocity = max(min(raw_velocity, velocity_limit), -velocity_limit)
        dxl_cmd = max(min(dxl_cmd, velocity_limit), -velocity_limit)
        # self.get_logger().info(f"Setting goal velocity to {dxl_cmd}, in motor node")
        self.controller.set_goal_velocity(dxl_cmd)

    def publish_velocity_callback(self):
        raw_velocity = self.controller.get_present_velocity()
        rpm = utils.raw_to_rpm(raw_velocity)
        linear_velocity = utils.rpm_to_linear_velocity_mps(rpm)
        msg = Float64()
        msg.data = linear_velocity
        self.velocity_pub.publish(msg)

class DynamixelMXController:
    # Control Table Addresses (from MX-106T/R manual)
    ADDR_TORQUE_ENABLE = 64
    ADDR_GOAL_VELOCITY = 104  # Velocity control address
    ADDR_PRESENT_VELOCITY = 128
    ADDR_PROFILE_ACCELERATION = 108
    ADDR_PROFILE_VELOCITY = 112
    ADDR_OPERATING_MODE = 11
    ADDR_MIN_POSITION_LIMIT = 52
    ADDR_MAX_POSITION_LIMIT = 48
    DXL_MINIMUM_LIMIT_VALUE = 0
    DXL_MAXIMUM_LIMIT_VALUE = 2500

    # Protocol and Defaults
    PROTOCOL_VERSION = 2.0
    BAUDRATE = 1000000
    TORQUE_ENABLE = 1
    TORQUE_DISABLE = 0
    VELOCITY_CONTROL_MODE = 1  # Operating mode for velocity control

    def __init__(self, device_name='/dev/ttyUSB0', motor_id=1):
        self.device_name = device_name
        self.motor_id = motor_id
        self.port_handler = PortHandler(device_name)
        self.packet_handler = PacketHandler(self.PROTOCOL_VERSION)

    def open_port(self):
        if not self.port_handler.openPort():
            raise Exception("Failed to open the port")
        if not self.port_handler.setBaudRate(self.BAUDRATE):
            raise Exception("Failed to set the baudrate")
        print("Port opened and baudrate set")

    def set_operating_mode(self):
        result, error = self.packet_handler.write1ByteTxRx(
            self.port_handler, self.motor_id, self.ADDR_OPERATING_MODE, self.VELOCITY_CONTROL_MODE
        )
        self.check_comm_result(result, error, "Set operating mode to velocity control")

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

    def set_goal_velocity(self, velocity):
        result, error = self.packet_handler.write4ByteTxRx(
            self.port_handler, self.motor_id, self.ADDR_GOAL_VELOCITY, int(velocity)
        )
        self.check_comm_result(result, error, "Set goal velocity")

    def get_present_velocity(self):
        velocity, result, error = self.packet_handler.read4ByteTxRx(
            self.port_handler, self.motor_id, self.ADDR_PRESENT_VELOCITY
        )
        self.check_comm_result(result, error, "Read present velocity")
        if velocity > 0x7FFFFFFF:
            velocity -= 4294967296  # Convert to signed integer
        return velocity

    def close_port(self):
        self.port_handler.closePort()
        print("Port closed")

    def check_comm_result(self, result, error, action):
        if result != 0:
            print(f"Communication error in {action}: {self.packet_handler.getTxRxResult(result)}")
        elif error != 0:
            print(f"Packet error in {action}: {self.packet_handler.getRxPacketError(error)}")
            
    #Set the acceleration and velocity limits
    
    def set_vel_and_accel(self, profile_acceleration=100, profile_velocity=100):
        self.PROFILE_ACCELERATION = profile_acceleration
        self.PROFILE_VELOCITY = profile_velocity
        
        result, error = self.packet_handler.write4ByteTxRx(
            self.port_handler, self.motor_id, self.ADDR_PROFILE_ACCELERATION, self.PROFILE_ACCELERATION
        )
        self.check_comm_result(result, error, "Set Profile Acceleration")
        result, error = self.packet_handler.write4ByteTxRx(
            self.port_handler, self.motor_id, self.ADDR_PROFILE_VELOCITY, self.PROFILE_VELOCITY
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

def main(args=None):
    rclpy.init(args=args)
    controller = DynamixelMXController()
    controller.open_port()
    controller.set_operating_mode()  # Switch to velocity control mode
    # controller.set_position_limits()
    controller.set_vel_and_accel()
    controller.enable_torque()
    node = MotorPublisherNode(controller)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # controller.set_goal_velocity(0)
        controller.disable_torque()
        controller.close_port()
        controller.timer.cancel()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()