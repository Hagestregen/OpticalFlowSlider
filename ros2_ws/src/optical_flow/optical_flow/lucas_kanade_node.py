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
        super().__init__('lucas_kanade_ST_clahe_node')
        
        # Declare parameters for width and height with default values
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        
        # Get the parameter values
        self.width = self.get_parameter('width').get_parameter_value().integer_value
        self.height = self.get_parameter('height').get_parameter_value().integer_value
        self.width_depth = 640
        self.height_depth = 480
        self.fps = 30
        self.pixel_to_meter = 0.001063  # placeholder gets updated in callback
        self.focal_length_x = None  # Will be set after receiving camera info
        self.writeCsv = True
        
        self.criteria_subpix = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 0.001)
        
        # Initialize CLAHE for contrast enhancement
        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        
        # Prepare CSV file for inference times
        if self.writeCsv:
            self.csv_filename = f"lk_ST_CLAHE_{self.width}x{self.height}.csv"
            if not os.path.isfile(self.csv_filename):
                with open(self.csv_filename, 'w', newline='') as csvfile:
                    writer = csv.writer(csvfile)
                    writer.writerow(['timestamp', 'inference_time_s'])
        
        # Create a QoS profile for images
        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST
        )

        # Subscribe to the image topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            qos_profile
        )
        
        self.sub_depth = self.create_subscription(
            Range, '/camera/depth/median_distance',
            self.depth_callback, 10
        )
        
        self.sub_camera_info = self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info',
            self.camera_info_callback, 10
        )

        # Publisher for the computed velocity vector with header
        self.velocity_pub = self.create_publisher(Vector3Stamped, '/optical_flow/LK_velocity', qos_profile)
        self.velocity_smooth_pub = self.create_publisher(Vector3Stamped, '/optical_flow/LK_smooth_velocity', qos_profile)

        self.bridge = CvBridge()
        self.prev_gray = None
        self.prev_points = None
        self.prev_stamp = None

        # Parameters for Shi-Tomasi feature detection
        self.feature_params = dict(
            maxCorners=500,
            qualityLevel=0.1,
            minDistance=10,
            blockSize=7
        )

        # Parameters for Lucas-Kanade optical flow
        self.lk_params = dict(
            winSize=(31, 31),
            maxLevel=3,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 50, 0.001)
        )
        
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
        if self.writeCsv:
            start_time = time.perf_counter()
        try:
            # Convert ROS Image to OpenCV image (assumes BGR8 encoding)
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")
            return

        # Convert the frame to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Apply CLAHE to enhance contrast
        gray = self.clahe.apply(gray)

        # Retrieve current timestamp from the image header (in seconds)
        current_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self.prev_gray is None:
            # For the first frame, initialize with CLAHE-enhanced image
            self.prev_gray = gray
            self.prev_points = cv2.goodFeaturesToTrack(gray, mask=None, **self.feature_params)
            if self.prev_points is not None:
                self.prev_points = cv2.cornerSubPix(gray, self.prev_points, (10, 10), (-1, -1), self.criteria_subpix)
            self.prev_stamp = current_stamp
            return

        # Calculate the time difference (dt) between frames
        dt = current_stamp - self.prev_stamp
        if dt <= 0:
            dt = 1e-3  # avoid division by zero

        # Calculate optical flow from the previous frame to the current frame
        next_points, status, error = cv2.calcOpticalFlowPyrLK(self.prev_gray, gray, self.prev_points, None, **self.lk_params)
        if next_points is None or status is None:
            self.get_logger().warn("No optical flow found; reinitializing features.")
            self.prev_points = cv2.goodFeaturesToTrack(gray, mask=None, **self.feature_params)
            if self.prev_points is not None:
                self.prev_points = cv2.cornerSubPix(gray, self.prev_points, (10, 10), (-1, -1), self.criteria_subpix)
            self.prev_gray = gray
            self.prev_stamp = current_stamp
            return

        # Select good points where tracking was successful
        good_new = next_points[status == 1]
        good_old = self.prev_points[status == 1]
        
        if len(good_new) < 50:  # Reinitialize if fewer than 50 points remain
            self.get_logger().info("Tracked points below threshold; reinitializing features.")
            self.prev_points = cv2.goodFeaturesToTrack(gray, mask=None, **self.feature_params)
            if self.prev_points is not None:
                self.prev_points = cv2.cornerSubPix(gray, self.prev_points, (10, 10), (-1, -1), self.criteria_subpix)
            self.prev_gray = gray
            self.prev_stamp = current_stamp
            return

        # Compute displacement vectors (in pixels) for each feature
        displacements = good_new - good_old  # shape: (n,2)
        if displacements.size > 0:
            median_disp = np.median(displacements, axis=0)  # Use median instead of mean
            velocity_pixels = median_disp / dt
        else:
            velocity_pixels = np.array([0.0, 0.0])

        # Convert the pixel velocity to physical units (meters per second)
        velocity_mps = velocity_pixels * self.pixel_to_meter

        # Create a Vector3Stamped message and populate header and vector
        vel_msg = Vector3Stamped()
        vel_msg.header = msg.header  # Use the same header from the image
        vel_msg.vector.x = float(velocity_mps[0])
        vel_msg.vector.y = float(velocity_mps[1])
        vel_msg.vector.z = 0.0  # assuming 2D motion

        # Publish the optical flow velocity with header
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

        # Update the previous frame, feature points, and timestamp for the next callback
        self.prev_gray = gray.copy()
        self.prev_points = good_new.reshape(-1, 1, 2)
        self.prev_stamp = current_stamp
        
        if self.writeCsv:
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


# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image, Range, CameraInfo
# from geometry_msgs.msg import Vector3Stamped
# import cv2
# import numpy as np
# from cv_bridge import CvBridge
# from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
# import os
# import csv
# import time
# from collections import deque

# class LucasKanadeNode(Node):
#     def __init__(self):
#         super().__init__('lucas_kanade_medium_node')
        
#         # --- Parameters ---
#         # self.width = 640
#         # self.height = 480
#         # Declare parameters for width and height with default values
#         self.declare_parameter('width', 640)
#         self.declare_parameter('height', 480)
        
#         # Get the parameter values
#         self.width = self.get_parameter('width').get_parameter_value().integer_value
#         self.height = self.get_parameter('height').get_parameter_value().integer_value
#         self.fps = 30
#         self.pixel_to_meter = 0.001063  # placeholder gets updated in callback
        
#         self.writeCsv = False
#         self.focal_length_x = None

#         # Error & track thresholds
#         self.err_thresh    = 4.0   # allow higher solver residual
#         self.fb_err_thresh = 6.0   # allow larger forward-backward error
#         self.min_tracks    = 20    # minimum survivors before reseeding

#         # Shi-Tomasi & LK params
#         self.feature_params = dict(
#             maxCorners=500,
#             qualityLevel=0.01,
#             minDistance=10,
#             blockSize=7
#         )
#         self.lk_params = dict(
#             winSize=(31, 31),
#             maxLevel=3,
#             criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 50, 0.001)
#         )
#         self.criteria_subpix = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 20, 0.0001)

#         # CSV logging setup
#         if self.writeCsv:
#             self.csv_filename = f"lk_medium_{self.width}x{self.height}.csv"
#             # Write header if new file
#             if not os.path.isfile(self.csv_filename):
#                 with open(self.csv_filename, 'w', newline='') as csvfile:
#                     writer = csv.writer(csvfile)
#                     writer.writerow(['timestamp', 'inference_time_s'])
#             # self.csv_filename = f"lk_optimized_{self.width}x{self.height}.csv"
#             # self.csv_file     = open(self.csv_filename, 'a', newline='')
#             # self.csv_writer   = csv.writer(self.csv_file)
#             # if os.path.getsize(self.csv_filename) == 0:
#             #     self.csv_writer.writerow(['timestamp', 'inference_time_s'])
#             # self._last_flush = time.time()

#         # QoS for image transport
#         qos = QoSProfile(
#             depth=10,
#             reliability=QoSReliabilityPolicy.BEST_EFFORT,
#             history=QoSHistoryPolicy.KEEP_LAST
#         )

#         # ROS subscriptions and publishers
#         self.subscription        = self.create_subscription(
#             Image,
#             '/camera/camera/color/image_raw',
#             self.image_callback,
#             qos
#             )
#         self.velocity_pub        = self.create_publisher(
#             Vector3Stamped,
#             '/optical_flow/LK_velocity',
#             qos
#             )
#         self.velocity_smooth_pub = self.create_publisher(
#             Vector3Stamped,
#             '/optical_flow/LK_smooth_velocity',
#             qos
#             )
        
#         self.sub_depth = self.create_subscription(
#             Range, '/camera/depth/median_distance',
#             self.depth_callback, 10
#             )
        
#         self.sub_camera_info = self.create_subscription(
#             CameraInfo, '/camera/camera/color/camera_info',
#             self.camera_info_callback, 10
#             )

#         # Bridge & CLAHE
#         self.bridge = CvBridge()
#         self.clahe  = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))

#         # State
#         self.prev_gray   = None
#         self.prev_points = None
#         self.prev_stamp  = None
#         self.velocity_buffer = deque(maxlen=5)

#         self.get_logger().info('Optimized Lucas–Kanade Optical Flow Node started.')

#     def _init_features(self, gray, stamp):
#         # Full reinitialization of feature set
#         self.prev_gray = gray
#         pts = cv2.goodFeaturesToTrack(gray, mask=None, **self.feature_params)
#         if pts is not None:
#             self.prev_points = cv2.cornerSubPix(
#                 gray, pts, (10,10), (-1,-1), self.criteria_subpix
#             )
#         else:
#             self.prev_points = None
#         self.prev_stamp = stamp
#         self.get_logger().warning('Full feature reinit')

#     def _add_new_features(self, good_pts, gray):
#         # Partial reseeding: add new features around surviving points
#         mask = np.ones_like(gray, dtype=np.uint8) * 255
#         for x, y in good_pts.astype(int):
#             cv2.circle(mask, (x,y), self.feature_params['minDistance'], 0, -1)
#         new_pts = cv2.goodFeaturesToTrack(gray, mask=mask, **self.feature_params)
#         if new_pts is not None:
#             new_pts = cv2.cornerSubPix(gray, new_pts, (10,10), (-1,-1), self.criteria_subpix)
#             all_pts = np.vstack([good_pts, new_pts.reshape(-1,2)])
#         else:
#             all_pts = good_pts
#         # Trim to maxCorners
#         max_n = self.feature_params['maxCorners']
#         if len(all_pts) > max_n:
#             all_pts = all_pts[:max_n]
#         return all_pts
    
    
#     def depth_callback(self, msg: Range):
#         depth = float(msg.range)
#         self.median_depth = depth
#         if self.focal_length_x:
#             self.pixel_to_meter = self.median_depth / self.focal_length_x
#             self.get_logger().info(f"Updated pixel_to_meter: {self.pixel_to_meter:.6f}")
            
#     def camera_info_callback(self, msg: CameraInfo):
#         if self.focal_length_x is None:
#             self.focal_length_x = msg.k[0]  # fx is the first element in the K matrix
#             self.get_logger().info(f"Received focal length fx = {self.focal_length_x:.2f} px")

#     def image_callback(self, msg: Image):
#         if self.writeCsv:
#             start_time = time.perf_counter()
#         try:
#             frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
#         except Exception as e:
#             self.get_logger().error(f"Error converting image: {e}")
#             return

#         # Preprocess
#         gray = cv2.GaussianBlur(
#             cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY),
#             (3,3), 0
#         )
#         gray = self.clahe.apply(gray)
#         ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

#         # First frame
#         if self.prev_gray is None:
#             pts = cv2.goodFeaturesToTrack(gray, mask=None, **self.feature_params)
#             self.prev_points = pts
#             self.prev_gray   = gray
#             self.prev_stamp  = ts
#             return
#         # Time delta
#         dt = max(ts - self.prev_stamp, 1e-3)

#         # Forward LK
#         p1, st1, err1 = cv2.calcOpticalFlowPyrLK(
#             self.prev_gray, gray, self.prev_points, None, **self.lk_params
#         )
#         if p1 is None or st1 is None or err1 is None:
#             self._init_features(gray, ts)
#             return

#         # Backward LK
#         p0r, st2, err2 = cv2.calcOpticalFlowPyrLK(
#             gray, self.prev_gray, p1, None, **self.lk_params
#         )
#         if p0r is None or st2 is None or err2 is None:
#             self._init_features(gray, ts)
#             return

#         # Build mask of good tracks
#         fb_err = np.linalg.norm(
#             self.prev_points.reshape(-1,2) - p0r.reshape(-1,2),
#             axis=1
#         )
#         mask = (
#             (st1.ravel()==1) &
#             (st2.ravel()==1) &
#             (err1.ravel() < self.err_thresh) &
#             (fb_err     < self.fb_err_thresh)
#         )
#         good_old = self.prev_points.reshape(-1,2)[mask]
#         good_new = p1.reshape(-1,2)[mask]

#         # Log survival
#         # self.get_logger().info(
#             # f"Tracks before filter: {len(self.prev_points)}, after: {len(good_new)}"
#         # )

#         # # Partial reseed if needed
#         # if len(good_new) < self.min_tracks:
#         #     good_new = self._add_new_features(good_new, gray)
#         #     self.get_logger().info(
#         #         f"After partial reseed: {len(good_new)} tracks"
#         #     )

#         # Partial reseed
#         if len(good_new) < self.min_tracks:
#             good_new = self._add_new_features(good_new, gray)
#             # self.get_logger().info(f"After partial reseed: {len(good_new)} tracks")
#             # Update state, skip velocity this frame
#             self.prev_points = good_new.reshape(-1,1,2)
#             self.prev_gray   = gray.copy()
#             self.prev_stamp  = ts
#             return

#         # Now we can safely RANSAC, because good_old & good_new match
#         if len(good_new) < 3:
#             vel_px = np.zeros(2)
#         else:
#             M, inliers = cv2.estimateAffinePartial2D(
#                 good_old, good_new,
#                 method=cv2.RANSAC,
#                 ransacReprojThreshold=5.0
#             )
#             if M is not None:
#                 dx, dy = M[0,2], M[1,2]
#                 vel_px = np.array([dx, dy]) / dt
#             else:
#                 vel_px = np.zeros(2)


#         vel_mps = vel_px * self.pixel_to_meter

#         # Publish raw
#         vel_msg = Vector3Stamped()
#         vel_msg.header = msg.header
#         vel_msg.vector.x = float(vel_mps[0])
#         vel_msg.vector.z = 0.0
#         self.velocity_pub.publish(vel_msg)

#         # Smooth
#         self.velocity_buffer.append(vel_px)
#         vel_smooth = np.mean(self.velocity_buffer, axis=0)
#         vel_smooth_msg = Vector3Stamped()
#         vel_smooth_msg.header = msg.header
#         vel_smooth_msg.vector.x = float(vel_smooth[0])
#         vel_smooth_msg.vector.z = 0.0
#         self.velocity_smooth_pub.publish(vel_smooth_msg)

#         # Update state
#         self.prev_points = good_new.reshape(-1,1,2)
#         self.prev_gray   = gray.copy()
#         self.prev_stamp  = ts
        
#         if self.writeCsv:
#                 end_time = time.perf_counter()
#                 inference_time = end_time - start_time
                
#                 # Append to CSV
#                 timestamp = time.time()
#                 with open(self.csv_filename, 'a', newline='') as csvfile:
#                     csv.writer(csvfile).writerow([timestamp, inference_time])
        
#         # # Log timing
#         # elapsed = time.perf_counter() - start
#         # now = time.time()
#         # self.csv_writer.writerow([now, elapsed])
#         # if now - self._last_flush > 10.0:
#         #     self.csv_file.flush()
#         #     self._last_flush = now

#     def destroy_node(self):
#         self.csv_file.close()
#         super().destroy_node()


# def main(args=None):
#     rclpy.init(args=args)
#     node = LucasKanadeNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.get_logger().info('Keyboard interrupt, shutting down.')
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()
#         cv2.destroyAllWindows()

# if __name__ == '__main__':
#     main()