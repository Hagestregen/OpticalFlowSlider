#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class InertialsenseAccelSubscriber(Node):
    def __init__(self):
        super().__init__('ineritalsense_accel_subscriber')
        
        # Create a QoS profile with BEST_EFFORT reliability to match the publisher.
        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST
        )
        
        # Subscriber for the IMU data from the camera with the custom QoS profile.
        self.subscription = self.create_subscription(
            Imu,
            '/inertialsense/imu',
            self.imu_callback,
            qos_profile
        )
        # Publisher for the linear_acceleration.x value using the same QoS profile.
        self.publisher = self.create_publisher(Float64, '/inertialsense_accel_x', qos_profile)
        self.get_logger().info('AccelSubscriber node has been started.')

    def imu_callback(self, msg: Imu):
        # Extract the x component of the linear acceleration.
        accel_x = msg.linear_acceleration.x
        # self.get_logger().info(f'Received linear_acceleration.x: {accel_x}')
        
        # Create and publish the Float64 message.
        out_msg = Float64()
        out_msg.data = accel_x
        self.publisher.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = InertialsenseAccelSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
