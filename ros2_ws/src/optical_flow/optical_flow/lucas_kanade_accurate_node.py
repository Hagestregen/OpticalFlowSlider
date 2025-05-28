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
        super().__init__('lucas_kanade_light_node')
        
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
        
        # CLAHE for contrast enhancement
        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        
        # Prepare CSV file for inference times
        if self.writeCsv:
            self.csv_filename = f"lk_ORB_{self.width}x{self.height}.csv"
            # Write header if new file
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
            # For the first frame, initialize with ORB
            self.prev_gray = gray
            orb = cv2.ORB_create(nfeatures=1000, fastThreshold=10)  # Increased nfeatures and lowered fastThreshold
            keypoints = orb.detect(gray, None)
            if keypoints:
                self.prev_points = np.array([kp.pt for kp in keypoints], dtype=np.float32).reshape(-1, 1, 2)
                self.prev_points = cv2.cornerSubPix(gray, self.prev_points, (10, 10), (-1, -1), self.criteria_subpix)
            else:
                self.prev_points = None
            self.prev_stamp = current_stamp
            return

        # Skip processing if no previous points are available
        if self.prev_points is None:
            self.get_logger().warn("No previous points available; skipping this frame.")
            return

        # Calculate the time difference (dt) between frames
        dt = current_stamp - self.prev_stamp
        if dt <= 0:
            dt = 1e-3  # avoid division by zero

        # Calculate optical flow from the previous frame to the current frame
        next_points, status, error = cv2.calcOpticalFlowPyrLK(self.prev_gray, gray, self.prev_points, None, **self.lk_params)
        if next_points is None or status is None:
            self.get_logger().warn("No optical flow found; reinitializing features.")
            orb = cv2.ORB_create(nfeatures=1000, fastThreshold=10)
            keypoints = orb.detect(gray, None)
            if keypoints:
                self.prev_points = np.array([kp.pt for kp in keypoints], dtype=np.float32).reshape(-1, 1, 2)
                self.prev_points = cv2.cornerSubPix(gray, self.prev_points, (10, 10), (-1, -1), self.criteria_subpix)
            else:
                self.prev_points = None
            self.prev_gray = gray
            self.prev_stamp = current_stamp
            return

        # Select good points where tracking was successful
        good_new = next_points[status == 1]
        good_old = self.prev_points[status == 1]
        
        if len(good_new) < 50:  # Reinitialize if fewer than 50 points remain
            self.get_logger().info("Tracked points below threshold; reinitializing features.")
            orb = cv2.ORB_create(nfeatures=1000, fastThreshold=10)
            keypoints = orb.detect(gray, None)
            if keypoints:
                self.prev_points = np.array([kp.pt for kp in keypoints], dtype=np.float32).reshape(-1, 1, 2)
                self.prev_points = cv2.cornerSubPix(gray, self.prev_points, (10, 10), (-1, -1), self.criteria_subpix)
            else:
                self.prev_points = None
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
# from concurrent.futures import ThreadPoolExecutor
# from numba import njit

# class LucasKanadeNode(Node):
#     def __init__(self):
#         super().__init__('lucas_kanade_heavy_node')
        
#         # --- Parameters ---
#         self.declare_parameter('width', 640)
#         self.declare_parameter('height', 480)
#         self.width = self.get_parameter('width').get_parameter_value().integer_value
#         self.height = self.get_parameter('height').get_parameter_value().integer_value
#         self.fps = 30
#         self.pixel_to_meter = 0.001063  # Updated in callback
#         self.num_threads = os.cpu_count()
        
#         self.writeCsv = True
#         self.focal_length_x = None

#         # Error & track thresholds (adaptive for robustness)
#         self.err_thresh = 1.0
#         self.fb_err_thresh = 2.0
#         self.min_tracks = 50

#         # Adjusted feature params for fewer, higher-quality features
#         self.feature_params = dict(
#             maxCorners=300,  # Reduced from 500 to focus on quality over quantity
#             qualityLevel=0.001,
#             minDistance=10,
#             blockSize=10
#         )
#         # Dynamic LK params based on resolution
#         win_size = (30, 30) if self.width <= 640 else (50, 50)
#         max_level = 3 if self.width <= 640 else 4
#         self.lk_params = dict(
#             winSize=win_size,
#             maxLevel=max_level,
#             criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 100, 0.0001)
#         )
#         self.criteria_subpix = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 20, 0.0001)

#         # CSV logging setup
#         if self.writeCsv:
#             self.csv_filename = f"lk_heavy_{self.width}x{self.height}.csv"
#             if not os.path.isfile(self.csv_filename):
#                 with open(self.csv_filename, 'w', newline='') as csvfile:
#                     writer = csv.writer(csvfile)
#                     writer.writerow(['timestamp', 'inference_time_s'])

#         # QoS profile
#         qos = QoSProfile(
#             depth=10,
#             reliability=QoSReliabilityPolicy.BEST_EFFORT,
#             history=QoSHistoryPolicy.KEEP_LAST
#         )

#         # ROS subscriptions and publishers
#         self.subscription = self.create_subscription(
#             Image, '/camera/camera/color/image_raw', self.image_callback, qos)
#         self.velocity_pub = self.create_publisher(
#             Vector3Stamped, '/optical_flow/LK_velocity', qos)
#         self.velocity_smooth_pub = self.create_publisher(
#             Vector3Stamped, '/optical_flow/LK_smooth_velocity', qos)
        
#         self.sub_depth = self.create_subscription(
#             Range, '/camera/depth/median_distance', self.depth_callback, 10)
#         self.sub_camera_info = self.create_subscription(
#             CameraInfo, '/camera/camera/color/camera_info', self.camera_info_callback, 10)

#         # Bridge & CLAHE with dynamic clip limit
#         self.bridge = CvBridge()
#         self.clahe = cv2.createCLAHE(clipLimit=1.0, tileGridSize=(4,4))

#         # State
#         self.prev_gray = None
#         self.prev_points = None
#         self.prev_stamp = None
#         self.velocity_buffer = deque(maxlen=10)  # Increased from 5 for smoother output

#         self.get_logger().info('Optimized Lucas–Kanade Optical Flow Node started.')

#     def _init_features(self, gray, stamp):
#         self.prev_gray = gray
#         orb = cv2.ORB_create(nfeatures=self.feature_params['maxCorners'])
#         keypoints = orb.detect(gray, None)
#         if keypoints:
#             self.prev_points = np.array([kp.pt for kp in keypoints], dtype=np.float32).reshape(-1, 1, 2)
#         else:
#             self.prev_points = None
#         self.prev_stamp = stamp
#         self.get_logger().warning('Full feature reinit')

#     def _add_new_features(self, good_pts, gray):
#         mask = np.ones_like(gray, dtype=np.uint8) * 255
#         for x, y in good_pts.astype(int):
#             cv2.circle(mask, (x, y), self.feature_params['minDistance'], 0, -1)
#         orb = cv2.ORB_create(nfeatures=self.feature_params['maxCorners'] - len(good_pts))
#         keypoints = orb.detect(gray, mask=mask)
#         if keypoints:
#             new_pts = np.array([kp.pt for kp in keypoints], dtype=np.float32)
#             all_pts = np.vstack([good_pts, new_pts])
#         else:
#             all_pts = good_pts
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
#             self.focal_length_x = msg.k[0]
#             self.get_logger().info(f"Received focal length fx = {self.focal_length_x:.2f} px")

#     def compute_lk(self, prev_img, next_img, points):
#         p, st, err = cv2.calcOpticalFlowPyrLK(
#             prev_img, next_img, points, None, 
#             flags=cv2.OPTFLOW_LK_GET_MIN_EIGENVALS, **self.lk_params)
#         return p, st, err
    
#     @staticmethod
#     @njit
#     def compute_fb_error(prev_points, p0r):
#         return np.sqrt(np.sum((prev_points - p0r) ** 2, axis=1))

#     # Census transform for texture filtering
#     def census_transform(self, img, window_size=3):
#         h, w = img.shape
#         half_window = window_size // 2
#         census = np.zeros((h, w), dtype=np.uint8)
#         for y in range(half_window, h - half_window):
#             for x in range(half_window, w - half_window):
#                 center = img[y, x]
#                 binary = 0
#                 bit_pos = 0
#                 for dy in range(-half_window, half_window + 1):
#                     for dx in range(-half_window, half_window + 1):
#                         if dy == 0 and dx == 0:
#                             continue
#                         binary |= (1 << bit_pos) if img[y + dy, x + dx] >= center else 0
#                         bit_pos += 1
#                 census[y, x] = binary
#         return census

#     def compute_texture_gradient(self, census):
#         h, w = census.shape
#         cg = np.zeros((h, w), dtype=np.float32)
#         for y in range(1, h):
#             for x in range(1, w):
#                 hamming_left = bin(census[y, x] ^ census[y, x - 1]).count('1')
#                 hamming_top = bin(census[y, x] ^ census[y - 1, x]).count('1')
#                 cg[y, x] = hamming_left + hamming_top
#         return cg

#     def filter_features(self, points, cg, threshold=2):
#         mask = []
#         for pt in points.reshape(-1, 2):
#             x, y = int(pt[0]), int(pt[1])
#             if 0 <= y < cg.shape[0] and 0 <= x < cg.shape[1]:
#                 mask.append(cg[y, x] > threshold)
#             else:
#                 mask.append(False)
#         return np.array(mask, dtype=bool)

#     def image_callback(self, msg: Image):
#         if self.writeCsv:
#             start_time = time.perf_counter()
#         try:
#             frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
#         except Exception as e:
#             self.get_logger().error(f"Error converting image: {e}")
#             return

#         # Color correction using gray world assumption
#         b, g, r = cv2.split(frame)
#         b_mean, g_mean, r_mean = np.mean(b), np.mean(g), np.mean(r)
#         avg_mean = (b_mean + g_mean + r_mean) / 3
#         b = np.clip(b * (avg_mean / b_mean), 0, 255).astype(np.uint8)
#         g = np.clip(g * (avg_mean / g_mean), 0, 255).astype(np.uint8)
#         r = np.clip(r * (avg_mean / r_mean), 0, 255).astype(np.uint8)
#         frame = cv2.merge((b, g, r))

#         # Preprocess with dynamic CLAHE
#         gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#         brightness = np.mean(gray)
#         if brightness < 50:  # Dynamic adjustment for low-light underwater scenes
#             self.clahe.setClipLimit(3.0)
#         else:
#             self.clahe.setClipLimit(1.0)
#         gray = self.clahe.apply(gray)
#         gray = cv2.GaussianBlur(gray, (1, 1), 0)

#         ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

#         if self.prev_gray is None:
#             self._init_features(gray, ts)
#             return
#         dt = max(ts - self.prev_stamp, 1e-3)

#         # Compute census transform and texture gradient
#         census = self.census_transform(gray)
#         cg = self.compute_texture_gradient(census)

#         # Multithreading for LK computation
#         if self.prev_points is not None and len(self.prev_points) > 0:
#             # Filter points based on texture gradient
#             texture_mask = self.filter_features(self.prev_points, cg, threshold=2)
#             filtered_points = self.prev_points[texture_mask]
#             if len(filtered_points) < self.min_tracks:
#                 filtered_points = self.prev_points  # Fallback to unfiltered if too few

#             chunk_size = max(10, len(filtered_points) // (self.num_threads * 2))  # Optimized chunk size
#             point_chunks = [filtered_points[i:i + chunk_size] for i in range(0, len(filtered_points), chunk_size)]

#             with ThreadPoolExecutor(max_workers=self.num_threads) as executor:
#                 results = list(executor.map(
#                     lambda pts: self.compute_lk(self.prev_gray, gray, pts), point_chunks))

#             p1 = np.vstack([r[0] for r in results if r[0] is not None])
#             st1 = np.vstack([r[1] for r in results if r[1] is not None])
#             err1 = np.vstack([r[2] for r in results if r[2] is not None])

#             # Backward check
#             point_chunks = [p1[i:i + chunk_size] for i in range(0, len(p1), chunk_size)]
#             with ThreadPoolExecutor(max_workers=self.num_threads) as executor:
#                 results = list(executor.map(
#                     lambda pts: self.compute_lk(gray, self.prev_gray, pts), point_chunks))

#             p0r = np.vstack([r[0] for r in results if r[0] is not None])
#             st2 = np.vstack([r[1] for r in results if r[1] is not None])
#             err2 = np.vstack([r[2] for r in results if r[2] is not None])
#         else:
#             self._init_features(gray, ts)
#             return

#         fb_err = self.compute_fb_error(filtered_points.reshape(-1, 2), p0r.reshape(-1, 2))

#         # Adaptive error thresholds
#         mask = (
#             (st1.ravel() == 1) &
#             (st2.ravel() == 1) &
#             (err1.ravel() < self.err_thresh) &
#             (fb_err < self.fb_err_thresh)
#         )
#         good_old = filtered_points.reshape(-1, 2)[mask]
#         good_new = p1.reshape(-1, 2)[mask]

#         if len(good_new) < self.min_tracks:
#             good_new = self._add_new_features(good_new, gray)
#             self.prev_points = good_new.reshape(-1, 1, 2)
#             self.prev_gray = gray.copy()
#             self.prev_stamp = ts
#             return

#         # Adjust thresholds dynamically
#         if len(good_new) < self.min_tracks // 2:
#             self.err_thresh = 1.5
#             self.fb_err_thresh = 3.0
#         else:
#             self.err_thresh = 1.0
#             self.fb_err_thresh = 2.0

#         # Hybrid velocity estimation
#         if len(good_new) < 3:
#             vel_px = np.zeros(2)
#         else:
#             M, inliers = cv2.estimateAffinePartial2D(
#                 good_old, good_new, method=cv2.RANSAC, ransacReprojThreshold=3.0)
#             if M is not None and np.sum(inliers) >= 3:
#                 dx, dy = M[0, 2], M[1, 2]
#                 vel_px = np.array([dx, dy]) / dt
#             else:
#                 displacements = good_new - good_old
#                 vel_px = np.median(displacements, axis=0) / dt

#         vel_mps = vel_px * self.pixel_to_meter

#         vel_msg = Vector3Stamped()
#         vel_msg.header = msg.header
#         vel_msg.vector.x = float(vel_mps[0])
#         vel_msg.vector.z = 0.0
#         self.velocity_pub.publish(vel_msg)

#         self.velocity_buffer.append(vel_px)
#         vel_smooth = np.mean(self.velocity_buffer, axis=0)
#         vel_smooth_msg = Vector3Stamped()
#         vel_smooth_msg.header = msg.header
#         vel_smooth_msg.vector.x = float(vel_smooth[0] * self.pixel_to_meter)
#         vel_smooth_msg.vector.z = 0.0
#         self.velocity_smooth_pub.publish(vel_smooth_msg)

#         self.prev_points = good_new.reshape(-1, 1, 2)
#         self.prev_gray = gray.copy()
#         self.prev_stamp = ts

#         if self.writeCsv:
#             end_time = time.perf_counter()
#             inference_time = end_time - start_time
#             timestamp = time.time()
#             with open(self.csv_filename, 'a', newline='') as csvfile:
#                 csv.writer(csvfile).writerow([timestamp, inference_time])

#     def destroy_node(self):
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
# from concurrent.futures import ThreadPoolExecutor
# from numba import njit

# class LucasKanadeNode(Node):
#     def __init__(self):
#         super().__init__('lucas_kanade_heavy_node')
        
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
#         self.pixel_to_meter = 0.001063  # Updated in callback
#         self.num_threads = os.cpu_count()  # Use all available CPU cores
        
#         self.writeCsv = False
#         self.focal_length_x = None

#         # Error & track thresholds (stricter for better accuracy)
#         self.err_thresh = 1.0  # Stricter solver residual
#         self.fb_err_thresh = 2.0  # Stricter forward-backward error
#         self.min_tracks = 50  # More survivors before reseeding

#         # Shi-Tomasi & LK params (optimized for accuracy and speed)
#         self.feature_params = dict(
#             maxCorners=500,  # More features
#             qualityLevel=0.001,  # Lower quality threshold
#             minDistance=10,
#             blockSize=10  # Larger block for stability
#         )
#         self.lk_params = dict(
#             winSize=(50, 50),  # Larger window
#             maxLevel=4,  # More pyramid levels
#             criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 100, 0.0001)  # Stricter termination
#         )
#         self.criteria_subpix = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 20, 0.0001)

#         # CSV logging setup
#         if self.writeCsv:
#             self.csv_filename = f"lk_heavy_{self.width}x{self.height}.csv"
#             if not os.path.isfile(self.csv_filename):
#                 with open(self.csv_filename, 'w', newline='') as csvfile:
#                     writer = csv.writer(csvfile)
#                     writer.writerow(['timestamp', 'inference_time_s'])

#         # QoS for image transport
#         qos = QoSProfile(
#             depth=10,
#             reliability=QoSReliabilityPolicy.BEST_EFFORT,
#             history=QoSHistoryPolicy.KEEP_LAST
#         )

#         # ROS subscriptions and publishers
#         self.subscription = self.create_subscription(
#             Image, '/camera/camera/color/image_raw', self.image_callback, qos)
#         self.velocity_pub = self.create_publisher(
#             Vector3Stamped, '/optical_flow/LK_velocity', qos)
#         self.velocity_smooth_pub = self.create_publisher(
#             Vector3Stamped, '/optical_flow/LK_smooth_velocity', qos)
        
#         self.sub_depth = self.create_subscription(
#             Range, '/camera/depth/median_distance', self.depth_callback, 10)
#         self.sub_camera_info = self.create_subscription(
#             CameraInfo, '/camera/camera/color/camera_info', self.camera_info_callback, 10)

#         # Bridge & CLAHE
#         self.bridge = CvBridge()
#         self.clahe = cv2.createCLAHE(clipLimit=1.0, tileGridSize=(4,4))  # Subtler enhancement

#         # State
#         self.prev_gray = None
#         self.prev_points = None
#         self.prev_stamp = None
#         self.velocity_buffer = deque(maxlen=5)

#         self.get_logger().info('Optimized Lucas–Kanade Optical Flow Node started.')

#     def _init_features(self, gray, stamp):
#         self.prev_gray = gray
#         # Use ORB for robust keypoint detection
#         orb = cv2.ORB_create(nfeatures=self.feature_params['maxCorners'])
#         keypoints = orb.detect(gray, None)
#         if keypoints:
#             self.prev_points = np.array([kp.pt for kp in keypoints], dtype=np.float32).reshape(-1, 1, 2)
#         else:
#             self.prev_points = None
#         self.prev_stamp = stamp
#         self.get_logger().warning('Full feature reinit')

#     def _add_new_features(self, good_pts, gray):
#         mask = np.ones_like(gray, dtype=np.uint8) * 255
#         for x, y in good_pts.astype(int):
#             cv2.circle(mask, (x, y), self.feature_params['minDistance'], 0, -1)
#         # Use ORB to detect new features
#         orb = cv2.ORB_create(nfeatures=self.feature_params['maxCorners'] - len(good_pts))
#         keypoints = orb.detect(gray, mask=mask)
#         if keypoints:
#             new_pts = np.array([kp.pt for kp in keypoints], dtype=np.float32)
#             all_pts = np.vstack([good_pts, new_pts])
#         else:
#             all_pts = good_pts
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
#             self.focal_length_x = msg.k[0]
#             self.get_logger().info(f"Received focal length fx = {self.focal_length_x:.2f} px")

#     def compute_lk(self, prev_img, next_img, points):
#         p, st, err = cv2.calcOpticalFlowPyrLK(
#             prev_img, next_img, points, None, 
#             flags=cv2.OPTFLOW_LK_GET_MIN_EIGENVALS, **self.lk_params)
#         return p, st, err
    
#     @staticmethod
#     @njit
#     def compute_fb_error(prev_points, p0r):
#         return np.sqrt(np.sum((prev_points - p0r) ** 2, axis=1))

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
#             cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), (1,1), 0)  # Minimal blur
        
#         gray = self.clahe.apply(gray)
#         # Normalize lighting to approximate handling of illumination changes
#         gray = cv2.normalize(gray, None, 0, 255, cv2.NORM_MINMAX)
#         ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

#         if self.prev_gray is None:
#             self._init_features(gray, ts)
#             return
#         dt = max(ts - self.prev_stamp, 1e-3)

#         # Multithreading for LK computation
#         if self.prev_points is not None and len(self.prev_points) > 0:
#             chunk_size = max(1, len(self.prev_points) // self.num_threads)
#             point_chunks = [self.prev_points[i:i + chunk_size] for i in range(0, len(self.prev_points), chunk_size)]

#             with ThreadPoolExecutor(max_workers=self.num_threads) as executor:
#                 results = list(executor.map(
#                     lambda pts: self.compute_lk(self.prev_gray, gray, pts), point_chunks))

#             p1 = np.vstack([r[0] for r in results if r[0] is not None])
#             st1 = np.vstack([r[1] for r in results if r[1] is not None])
#             err1 = np.vstack([r[2] for r in results if r[2] is not None])

#             # Backward check with multithreading
#             point_chunks = [p1[i:i + chunk_size] for i in range(0, len(p1), chunk_size)]
#             with ThreadPoolExecutor(max_workers=self.num_threads) as executor:
#                 results = list(executor.map(
#                     lambda pts: self.compute_lk(gray, self.prev_gray, pts), point_chunks))

#             p0r = np.vstack([r[0] for r in results if r[0] is not None])
#             st2 = np.vstack([r[1] for r in results if r[1] is not None])
#             err2 = np.vstack([r[2] for r in results if r[2] is not None])
#         else:
#             self._init_features(gray, ts)
#             return

#         # Use Numba-optimized forward-backward error computation
#         fb_err = self.compute_fb_error(self.prev_points.reshape(-1, 2), p0r.reshape(-1, 2))

#         # Build mask of good tracks with stricter thresholds
#         mask = (
#             (st1.ravel() == 1) &
#             (st2.ravel() == 1) &
#             (err1.ravel() < self.err_thresh) &
#             (fb_err < self.fb_err_thresh)
#         )
#         good_old = self.prev_points.reshape(-1, 2)[mask]
#         good_new = p1.reshape(-1, 2)[mask]

#         if len(good_new) < self.min_tracks:
#             good_new = self._add_new_features(good_new, gray)
#             self.prev_points = good_new.reshape(-1, 1, 2)
#             self.prev_gray = gray.copy()
#             self.prev_stamp = ts
#             return

#         if len(good_new) < 3:
#             vel_px = np.zeros(2)
#         else:
#             M, inliers = cv2.estimateAffinePartial2D(
#                 good_old, good_new, method=cv2.RANSAC, ransacReprojThreshold=3.0)
#             if M is not None:
#                 dx, dy = M[0, 2], M[1, 2]
#                 vel_px = np.array([dx, dy]) / dt
#             else:
#                 vel_px = np.zeros(2)

#         vel_mps = vel_px * self.pixel_to_meter

#         vel_msg = Vector3Stamped()
#         vel_msg.header = msg.header
#         vel_msg.vector.x = float(vel_mps[0])
#         vel_msg.vector.z = 0.0
#         self.velocity_pub.publish(vel_msg)

#         self.velocity_buffer.append(vel_px)
#         vel_smooth = np.mean(self.velocity_buffer, axis=0)
#         vel_smooth_msg = Vector3Stamped()
#         vel_smooth_msg.header = msg.header
#         vel_smooth_msg.vector.x = float(vel_smooth[0])
#         vel_smooth_msg.vector.z = 0.0
#         self.velocity_smooth_pub.publish(vel_smooth_msg)

#         self.prev_points = good_new.reshape(-1, 1, 2)
#         self.prev_gray = gray.copy()
#         self.prev_stamp = ts

#         if self.writeCsv:
#             end_time = time.perf_counter()
#             inference_time = end_time - start_time
#             timestamp = time.time()
#             with open(self.csv_filename, 'a', newline='') as csvfile:
#                 csv.writer(csvfile).writerow([timestamp, inference_time])

#     def destroy_node(self):
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