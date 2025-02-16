#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class AccelSubscriber(Node):
    def __init__(self):
        super().__init__('accel_subscriber')
        
        # Create a QoS profile with BEST_EFFORT reliability to match the publisher.
        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST
        )
        
        # Subscriber for the IMU data from the camera with the custom QoS profile.
        self.subscription = self.create_subscription(
            Imu,
            '/camera/camera/accel/sample',
            self.imu_callback,
            qos_profile
        )
        # Publisher for the linear_acceleration.x value using the same QoS profile.
        self.publisher = self.create_publisher(Float64, '/realsense_accel_x', qos_profile)
        self.vel_publisher = self.create_publisher(Float64, '/realsense_vel_x', qos_profile)
        self.get_logger().info('AccelSubscriber node has been started.')

        # Variables for integration.
        self.last_time = None   # Last timestamp (in seconds)
        self.velocity_x = 0.0   # Integrated velocity in m/s

        # To remove the bias, subtract 0.033.
        self.bias = 0.033  # m/s²

    def imu_callback(self, msg: Imu):
        # Extract the x component of the linear acceleration.
        raw_accel_x = msg.linear_acceleration.x

        #make realsense imu x direction same as inertialsense
        inverted_accel = -raw_accel_x
        # self.get_logger().info(f'Received linear_acceleration.x: {accel_x}')

        accel_x = inverted_accel - self.bias #remove bias from the accel data
        
        # Create and publish the Float64 message.
        out_msg = Float64()
        out_msg.data = accel_x
        self.publisher.publish(out_msg)

        # Compute current time from the header.
        # Convert sec and nanosec to a float in seconds.
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        if self.last_time is None:
            dt = 0.0  # No integration on the first message.
        else:
            dt = current_time - self.last_time
        
        # Save the current time for next callback.
        self.last_time = current_time
        
        # Integrate acceleration to get velocity.
        self.velocity_x += accel_x * dt
        
        # Publish the integrated velocity.
        vel_msg = Float64()
        vel_msg.data = self.velocity_x
        self.vel_publisher.publish(vel_msg)

def main(args=None):
    rclpy.init(args=args)
    node = AccelSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
