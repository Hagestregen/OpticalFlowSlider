import cv2
import rclpy
import pyrealsense2 as rs

import rclpy
from rclpy.node import Node
import pyrealsense2 as rs
from std_msgs.msg import Float32
import numpy as np

class DepthCalculationNode(Node):
    def __init__(self):
        super().__init__('depth_calculation_node')
        
        # Initialize RealSense pipeline
        try:
            self.pipeline = rs.pipeline()
            config = rs.config()
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            self.pipeline.start(config)
        
        except Exception as e:
            self.get_logger().info(e)
            pass
        
        # Get camera intrinsics (focal length)
        profile = self.pipeline.get_active_profile()
        depth_sensor = profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()
        intrinsics = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
        self.focal_length_x = intrinsics.fx
        
        # Publisher for conversion factor
        self.publisher = self.create_publisher(Float32, '/optical_flow/conversion_factor', 10)
        
        # Timer to periodically publish conversion factor
        self.timer = self.create_timer(1.0, self.calculate_and_publish)  # Update at 1 Hz
        
    def calculate_and_publish(self):
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        if not depth_frame:
            return
        
        # Convert depth frame to numpy array
        depth_image = np.asanyarray(depth_frame.get_data())
        
        # Define a central ROI (e.g., 100x100 pixels)
        h, w = depth_image.shape
        roi = depth_image[h//2-50:h//2+50, w//2-50:w//2+50]
        
        # Calculate median depth in meters
        valid_depths = roi[roi > 0]  # Filter out invalid zeros
        if len(valid_depths) == 0:
            self.get_logger().warn("No valid depth data in ROI")
            return
        median_depth = np.median(valid_depths) * self.depth_scale
        self.get_logger().info(f"Median depth: {median_depth}")
        
        # Calculate conversion factor (meters per pixel)
        conversion_factor = median_depth / self.focal_length_x
        
        # Publish the conversion factor
        msg = Float32()
        msg.data = float(conversion_factor)
        self.publisher.publish(msg)
        self.get_logger().info(f"Published conversion factor: {conversion_factor}")

def main(args=None):
    rclpy.init(args=args)
    node = DepthCalculationNode()
    rclpy.spin(node)
    node.pipeline.stop()
    rclpy.shutdown()

if __name__ == '__main__':
    main()




