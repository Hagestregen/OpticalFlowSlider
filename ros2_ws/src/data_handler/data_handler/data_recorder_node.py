#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray
import csv
import sys

class DataRecorderNode(Node):
    def __init__(self):
        super().__init__('data_recorder')
        
        # Open combined CSV file for writing
        self.combined_csv = open('combined_data.csv', 'w', newline='')
        
        # Initialize CSV writer
        self.combined_writer = csv.writer(self.combined_csv)
        
        # Write header for combined data
        self.combined_writer.writerow(['timestamp', 'velocity', 'accel_x'])
        
        # Storage for latest data
        self.latest_velocity = 0.0
        self.latest_accel_x = 0.0
        
        # Subscriptions
        self.imu_sub = self.create_subscription(
            Imu,
            '/inertialsense/imu',
            self.imu_callback,
            10
        )
        self.velocity_sub = self.create_subscription(
            Float64MultiArray,
            '/motor/present_velocity',
            self.velocity_callback,
            10
        )
        
        self.get_logger().info("DataRecorderNode started, recording to combined_data.csv")

    def imu_callback(self, msg):
        # Use current time as timestamp when message arrives
        timestamp = self.get_clock().now().nanoseconds / 1e9
        accel_x = msg.linear_acceleration.x
        # Update latest IMU data and write to CSV immediately
        self.latest_accel_x = accel_x
        self.combined_writer.writerow([timestamp, self.latest_velocity, self.latest_accel_x])

    def velocity_callback(self, msg):
        # Use current time as timestamp when message arrives
        timestamp = self.get_clock().now().nanoseconds / 1e9
        if len(msg.data) >= 2:
            velocity = msg.data[0]  # First element is velocity
            # Update latest velocity and write to CSV immediately
            self.latest_velocity = velocity
            self.combined_writer.writerow([timestamp, self.latest_velocity, self.latest_accel_x])
        else:
            self.get_logger().warn("Received malformed Float64MultiArray message")

    def shutdown(self):
        # Close the combined CSV file on shutdown
        self.combined_csv.close()
        self.get_logger().info("Combined CSV file closed, shutting down")

def main(args=None):
    rclpy.init(args=args)
    node = DataRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.shutdown()
        rclpy.shutdown()
        sys.exit(0)

if __name__ == '__main__':
    main()

# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Imu
# from std_msgs.msg import Float64MultiArray
# import csv
# import sys

# class DataRecorderNode(Node):
#     def __init__(self):
#         super().__init__('data_recorder')
        
#         # Open CSV files for writing
#         self.imu_csv = open('imu_data.csv', 'w', newline='')
#         self.velocity_csv = open('velocity_data.csv', 'w', newline='')
#         self.combined_csv = open('combined_data.csv' 'w', newline='')
        
#         # Initialize CSV writers
#         self.imu_writer = csv.writer(self.imu_csv)
#         self.velocity_writer = csv.writer(self.velocity_csv)
#         self.combined_csv_writer = csv.writer(self.combined_csv)
        
#         # Write headers
#         self.imu_writer.writerow(['timestamp', 'accel_x'])
#         self.velocity_writer.writerow(['timestamp', 'velocity'])
        
#         # Subscriptions
#         self.imu_sub = self.create_subscription(
#             Imu,
#             '/inertialsense/imu',
#             self.imu_callback,
#             10
#         )
#         self.velocity_sub = self.create_subscription(
#             Float64MultiArray,
#             '/motor/present_velocity',
#             self.velocity_callback,
#             10
#         )
        
#         self.get_logger().info("DataRecorderNode started, recording to imu_data.csv and velocity_data.csv")


#     def imu_callback(self, msg):
#         # Extract timestamp and acceleration data
#         timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
#         accel_x = msg.linear_acceleration.x

        
#         # Write to CSV
#         self.imu_writer.writerow([timestamp, accel_x])

#     def velocity_callback(self, msg):
#         # Extract timestamp and velocity from Float64MultiArray (assuming [velocity, timestamp])
#         if len(msg.data) >= 2:
#             velocity = msg.data[0]  # First element is velocity
#             timestamp = msg.data[1]  # Second element is timestamp
#             # Write to CSV
#             self.velocity_writer.writerow([timestamp, velocity])
#         else:
#             self.get_logger().warn("Received malformed Float64MultiArray message")

#     def shutdown(self):
#         # Close CSV files on shutdown
#         self.imu_csv.close()
#         self.velocity_csv.close()
#         self.combined_csv.close()
#         self.get_logger().info("CSV files closed, shutting down")

# def main(args=None):
#     rclpy.init(args=args)
#     node = DataRecorderNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.shutdown()
#         rclpy.shutdown()
#         sys.exit(0)

# if __name__ == '__main__':
#     main()