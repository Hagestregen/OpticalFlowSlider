#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float64
import cv2
import numpy as np
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import os
import csv
from ament_index_python.packages import get_package_share_directory
import time
import pyrealsense2 as rs
from sensor_msgs.msg import Range, CameraInfo
from collections import deque

class LucasKanadeNode(Node):
    def __init__(self):
        super().__init__('lucas_kanade_node')
        
        self.width = 640
        self.height = 480
        self.width_depth = 640
        self.height_depth = 480
        self.fps = 30
        self.pixel_to_meter = 0.001063
        self.focal_length_x = None  # Will be set after receiving camera info
         
        
        self.criteria_subpix = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 0.001)

        
        # Prepare CSV file for inference times
        self.csv_filename = f"lk_inference_light_{self.width}x{self.height}.csv"
        # Write header if new file
        if not os.path.isfile(self.csv_filename):
            with open(self.csv_filename, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(['timestamp', 'inference_time_s'])
                
        # --- RealSense intrinsics only (no frame loop) ---
        self.get_logger().info('Starting RealSense pipeline to read intrinsics…')
        pipeline = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, self.width, self.height, rs.format.rgb8, self.fps)
        cfg.enable_stream(rs.stream.depth, self.width_depth, self.height_depth, rs.format.z16, self.fps)
        profile = pipeline.start(cfg)
        color_profile = profile.get_stream(rs.stream.color).as_video_stream_profile()
        intr = color_profile.get_intrinsics()
        self.focal_length_x = intr.fx
        self.get_logger().info(f'RealSense fx = {self.focal_length_x:.2f} px')
        pipeline.stop()
        # ----------------------------------------------
                
                
        # Create a QoS profile for images.
        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST
        )

        # Subscribe to the image topic.
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            qos_profile
        )

        # Publisher for the computed velocity vector with header.
        self.velocity_pub = self.create_publisher(Vector3Stamped, '/optical_flow/LK_velocity', qos_profile) #Lucas-Kanade
        self.velocity_smooth_pub = self.create_publisher(Vector3Stamped, '/optical_flow/LK_smooth_velocity', qos_profile)

        self.bridge = CvBridge()
        self.prev_gray = None
        self.prev_points = None
        self.prev_stamp = None

        # Parameters for Shi-Tomasi feature detection.
        self.feature_params = dict(
            maxCorners=500,
            qualityLevel=0.1,
            minDistance=5,
            blockSize=7
        )

        # Parameters for Lucas-Kanade optical flow.
        self.lk_params = dict(
            winSize=(15, 15),
            maxLevel=2,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03)
        )
        
                # Depth subscription (Range message)
        self.median_depth = None
        self.sub_depth = self.create_subscription(
            Range,
            '/camera/depth/median_distance',
            self.depth_callback,
            10
        )
        
        self.sub_camera_info = self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info', self.camera_info_callback, 10)
        
        # Buffer for smoothing
        self.velocity_buffer = deque(maxlen=5)

        self.get_logger().info('Lucas-Kanade Optical Flow Node has been started.')
        
    def camera_info_callback(self, msg: CameraInfo):
        if self.focal_length_x is None:
            self.focal_length_x = msg.k[0]  # fx is the first element in the K matrix
            self.get_logger().info(f"Received focal length fx = {self.focal_length_x:.2f} px")
    
    def depth_callback(self, msg: Range):
        # Treat Range.range as depth in meters
        self.median_depth = float(msg.range)
        if self.focal_length_x:
            self.pixel_to_meter = self.median_depth / self.focal_length_x
            self.get_logger().info(
                f"Updated pixel_to_meter: {self.pixel_to_meter:.6f} m/px"
            )

    def image_callback(self, msg: Image):
        start_time = time.perf_counter()
        try:
            # Convert ROS Image to OpenCV image (assumes BGR8 encoding).
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")
            return

        # Convert the frame to grayscale.
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Retrieve current timestamp from the image header (in seconds).
        current_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        
        
        if self.prev_gray is None:
            # For the first frame, initialize the previous image, features, and timestamp.
            self.prev_gray = gray
            self.prev_points = cv2.goodFeaturesToTrack(gray, mask=None, **self.feature_params)
            self.prev_points = cv2.cornerSubPix(gray, self.prev_points, (10, 10), (-1, -1), self.criteria_subpix)
            self.prev_stamp = current_stamp
            return

        # Calculate the time difference (dt) between frames.
        dt = current_stamp - self.prev_stamp
        if dt <= 0:
            dt = 1e-3  # avoid division by zero

        # Calculate optical flow from the previous frame to the current frame.
        next_points, status, error = cv2.calcOpticalFlowPyrLK(self.prev_gray, gray, self.prev_points, None, **self.lk_params)
        if next_points is None or status is None:
            self.get_logger().warn("No optical flow found; reinitializing features.")
            self.prev_points = cv2.goodFeaturesToTrack(gray, mask=None, **self.feature_params)
            self.prev_points = cv2.cornerSubPix(gray, self.prev_points, (10, 10), (-1, -1), self.criteria_subpix)
            self.prev_gray = gray
            self.prev_stamp = current_stamp
            return

        # Select good points where tracking was successful.
        good_new = next_points[status == 1]
        good_old = self.prev_points[status == 1]
        
        if len(good_new) < 50:  # Reinitialize if fewer than 50 points remain
            self.get_logger().info("Tracked points below threshold; reinitializing features.")
            self.prev_points = cv2.goodFeaturesToTrack(gray, mask=None, **self.feature_params)
            self.prev_points = cv2.cornerSubPix(gray, self.prev_points, (10, 10), (-1, -1), self.criteria_subpix)
            self.prev_gray = gray
            self.prev_stamp = current_stamp
            return

        # Compute displacement vectors (in pixels) for each feature.
        displacements = good_new - good_old  # shape: (n,2)
        # if displacements.size == 0:
        #     mean_disp = np.array([0.0, 0.0])
        # else:
        #     # Compute the mean displacement vector.
        #     mean_disp = np.mean(displacements, axis=0)
        if displacements.size > 0:
            median_disp = np.median(displacements, axis=0)  # Use median instead of mean
            velocity_pixels = median_disp / dt
        else:
            velocity_pixels = np.array([0.0, 0.0])

        # Compute the velocity in pixels per second.
        # velocity_pixels = mean_disp / dt
        # velocity_pixels = velocity_pixels / dt

        # Convert the pixel velocity to physical units (meters per second).
        velocity_mps = velocity_pixels * self.pixel_to_meter

        # Create a Vector3Stamped message and populate header and vector.
        vel_msg = Vector3Stamped()
        vel_msg.header = msg.header  # Use the same header from the image
        vel_msg.vector.x = float(velocity_mps[0])
        vel_msg.vector.y = float(velocity_mps[1])
        vel_msg.vector.z = 0.0  # assuming 2D motion

        # Publish the optical flow velocity with header.
        self.velocity_pub.publish(vel_msg)
        
        
        # Smooth velocity
        self.velocity_buffer.append(velocity_mps)
        smoothed_velocity = sum(self.velocity_buffer) / len(self.velocity_buffer)
        
        vel_smooth_msg = Vector3Stamped()
        vel_smooth_msg.header = msg.header
        vel_smooth_msg.vector.x = float(smoothed_velocity[0])
        vel_smooth_msg.vector.y = float(smoothed_velocity[1])
        vel_smooth_msg.vector.z = 0.0
        
        self.velocity_smooth_pub.publish(vel_smooth_msg)
        
        
        # self.get_logger().info(f"Published optical flow velocity: {vel_msg.vector.x:.3f} m/s")

        # (Optional) Visualization: draw optical flow tracks.
        # mask = np.zeros_like(frame)
        # for new, old in zip(good_new, good_old):
        #     a, b = new.ravel().astype(int)
        #     c, d = old.ravel().astype(int)
        #     mask = cv2.line(mask, (a, b), (c, d), (0, 255, 0), 2)
        #     frame = cv2.circle(frame, (a, b), 5, (0, 0, 255), -1)
        # output = cv2.add(frame, mask)
        # cv2.imshow("Lucas-Kanade Optical Flow", output)
        # cv2.waitKey(1)

        # Update the previous frame, feature points, and timestamp for the next callback.
        self.prev_gray = gray.copy()
        self.prev_points = good_new.reshape(-1, 1, 2)
        self.prev_stamp = current_stamp
        
        end_time = time.perf_counter()
        inference_time = end_time - start_time
        timestamp = time.time()
        with open(self.csv_filename, 'a', newline='') as csvfile:
            csv.writer(csvfile).writerow([timestamp, inference_time])
            
            

def main(args=None):
    rclpy.init(args=args)
    node = LucasKanadeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()


