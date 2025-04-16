#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class InertialsenseAccelSubscriber(Node):
    def __init__(self):
        super().__init__('inertialsense_accel_subscriber')

        # self.accel_bias_x = 0.087
        
        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST
        )
        
        # Subscribe to IMU data.
        self.subscription = self.create_subscription(
            Imu,
            '/inertialsense/imu',
            self.imu_callback,
            qos_profile
        )
        # Publisher for the instantaneous acceleration x.
        self.accel_pub = self.create_publisher(Float64, '/inertialsense_accel_x', qos_profile)
        # Publisher for the integrated velocity in x.
        self.velocity_pub = self.create_publisher(Float64, '/inertialsense/velocity_x', qos_profile)
        self.velocity_no_bias_pub = self.create_publisher(Float64, '/inertialsense/velocity_no_bias', qos_profile)
        
        self.get_logger().info('AccelSubscriber node has been started.')

        # Variables for integration.
        self.last_time = None   # Last timestamp (in seconds)
        self.velocity_x = 0.0   # Integrated velocity in m/s
        self.velocity_no_bias_x = 0.0

        # To remove the bias, subtract 0.033.
        self.bias = 0.0815   # m/s²

    def imu_callback(self, msg: Imu):
        # Extract the x component of linear acceleration.
        raw_accel_x = msg.linear_acceleration.x
        # raw_accel_x = msg.linear_acceleration.x
        accel_x = raw_accel_x - self.bias
        
        
        # Publish the raw acceleration.
        accel_msg = Float64()
        accel_msg.data = accel_x
        self.accel_pub.publish(accel_msg)
        
        # Compute current time from the header.
        # Convert sec and nanosec to a float in seconds.
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        # self.get_logger().info(f'Current time: {current_time:.3f} s')
        
        if self.last_time is None:
            dt = 0.0  # No integration on the first message.
        else:
            dt = current_time - self.last_time
            # self.get_logger().info(f'Delta time: {dt:.3f} s')
        
        # Save the current time for next callback.
        self.last_time = current_time
        
        # Integrate acceleration to get velocity.
        self.velocity_x += accel_x * dt
        
        self.velocity_no_bias_x += raw_accel_x * dt
        
        # Publish the integrated velocity.
        vel_msg = Float64()
        vel_msg.data = self.velocity_x
        self.velocity_pub.publish(vel_msg)
        
        vel_no_bias_msg = Float64()
        vel_no_bias_msg.data = self.velocity_no_bias_x
        self.velocity_no_bias_pub.publish(vel_no_bias_msg)
        
        # Optionally, log the integrated velocity.
        # self.get_logger().info(f'Integrated velocity x: {self.velocity_x:.3f} m/s')

 

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
