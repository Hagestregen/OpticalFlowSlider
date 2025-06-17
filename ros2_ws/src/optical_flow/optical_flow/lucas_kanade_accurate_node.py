#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import Range, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from collections import deque
import time
import os
import csv

# ==============================
# Lucas-Kanade with Adaptive CLAHE & CSV Logging
# ==============================
class LucasKanadeNode(Node):
    def __init__(self):
        super().__init__('lucas_kanade_node')
        # Parameters
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('max_speed', 0.6)
        self.declare_parameter('apply_CLAHE', True)
        self.declare_parameter('clip_min', 1.0)
        self.declare_parameter('clip_max', 4.0)
        self.declare_parameter('C_min', 5.0)
        self.declare_parameter('C_max', 50.0)
        self.declare_parameter('tile_grid', [8, 8])
        self.declare_parameter('apply_gaussian', False)
        self.declare_parameter('apply_gamma', False)
        self.declare_parameter('gamma', 1.2)
        self.declare_parameter('apply_retinex', False)
        self.declare_parameter('writeCsv', False)

        # Retrieve parameters
        self.max_speed = self.get_parameter('max_speed').value
        self.apply_CLAHE = self.get_parameter('apply_CLAHE').value
        self.clip_min = self.get_parameter('clip_min').value
        self.clip_max = self.get_parameter('clip_max').value
        self.C_min = self.get_parameter('C_min').value
        self.C_max = self.get_parameter('C_max').value
        grid = tuple(self.get_parameter('tile_grid').value)
        self.apply_gaussian = self.get_parameter('apply_gaussian').value
        self.apply_gamma = self.get_parameter('apply_gamma').value
        self.gamma = self.get_parameter('gamma').value
        self.apply_retinex = self.get_parameter('apply_retinex').value
        self.writeCsv = self.get_parameter('writeCsv').value

        # CLAHE (initial clipLimit)
        self.clahe = cv2.createCLAHE(clipLimit=self.clip_min, tileGridSize=grid)

        # CSV setup
        if self.writeCsv:
            self.csv_filename = f"lk_adaptive_{self.get_parameter('width').value}x{self.get_parameter('height').value}.csv"
            if not os.path.isfile(self.csv_filename):
                with open(self.csv_filename, 'w', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow(['timestamp', 'inference_time_s'])

        # ROS and CV setup
        self.bridge = CvBridge()
        qos = 10
        self.create_subscription(Image, '/camera/camera/color/image_raw', self.image_callback, qos)
        self.create_subscription(Range, '/camera/depth/median_distance', self.depth_callback, qos)
        self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.camera_info_callback, qos)
        self.velocity_pub = self.create_publisher(Vector3Stamped, '/optical_flow/LK_velocity', qos)
        self.velocity_smooth_pub = self.create_publisher(Vector3Stamped, '/optical_flow/LK_smooth_velocity', qos)

        # Optical flow params
        self.feature_params = dict(maxCorners=500, qualityLevel=0.1,
                                   minDistance=10, blockSize=7)
        self.lk_params = dict(winSize=(31,31), maxLevel=3,
                              criteria=(cv2.TERM_CRITERIA_EPS|
                                        cv2.TERM_CRITERIA_COUNT, 50, 0.001))
        self.criteria_subpix = (cv2.TERM_CRITERIA_EPS +
                                cv2.TERM_CRITERIA_MAX_ITER, 10, 0.001)

        # State
        self.prev_gray = None
        self.prev_points = None
        self.prev_stamp = None
        self.pixel_to_meter = 0.001
        self.focal_length_x = None
        self.median_depth = None
        self.velocity_buffer = deque(maxlen=5)

        self.get_logger().info('Lucas-Kanade node with adaptive CLAHE & CSV logging started')

    def camera_info_callback(self, msg: CameraInfo):
        if self.focal_length_x is None:
            self.focal_length_x = msg.k[0]
            self.get_logger().info(f'Received fx={self.focal_length_x:.2f}')

    def depth_callback(self, msg: Range):
        self.median_depth = float(msg.range)
        if self.focal_length_x:
            self.pixel_to_meter = self.median_depth / self.focal_length_x

    def apply_retinex(self, img):
        # Placeholder for retinex
        return img

    def gamma_correction(self, gray, gamma):
        inv_gamma = 1.0 / gamma
        table = np.array([
            ((i/255.0)**inv_gamma)*255 for i in range(256)
        ]).astype('uint8')
        return cv2.LUT(gray, table)

    def image_callback(self, msg: Image):
        # Start timing
        if self.writeCsv:
            start_time = time.perf_counter()

        # Convert
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Adaptive CLAHE
        if self.apply_CLAHE:
            contrast = np.std(gray)/(np.mean(gray)+1e-3)
            clip = np.clip(
                self.clip_min + (contrast - self.C_min)/
                (self.C_max-self.C_min)*(self.clip_max-self.clip_min),
                self.clip_min, self.clip_max)
            self.clahe.setClipLimit(clip)
            gray = self.clahe.apply(gray)
        # Other preproc
        if self.apply_retinex:
            gray = self.apply_retinex(gray)
        if self.apply_gamma:
            gray = self.gamma_correction(gray, self.gamma)
        if self.apply_gaussian:
            gray = cv2.GaussianBlur(gray, (3,3), 0)

        # Timestamp
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        # If no prior points, initialize features
        if self.prev_points is None:
            pts = cv2.goodFeaturesToTrack(gray, mask=None, **self.feature_params)
            if pts is not None:
                self.prev_points = cv2.cornerSubPix(gray, pts, (10,10), (-1,-1), self.criteria_subpix)
            self.prev_gray = gray
            self.prev_stamp = stamp
            return

        dt = max(stamp - self.prev_stamp, 1e-3)
        # Calc flow only when points exist
        next_pts, status, _ = cv2.calcOpticalFlowPyrLK(
            self.prev_gray, gray, self.prev_points, None, **self.lk_params)
        # Re-init on error or no points
        if next_pts is None or status is None:
            pts = cv2.goodFeaturesToTrack(gray, mask=None, **self.feature_params)
            if pts is not None:
                self.prev_points = cv2.cornerSubPix(gray, pts, (10,10), (-1,-1), self.criteria_subpix)
            self.prev_gray = gray
            self.prev_stamp = stamp
            return

        good_new = next_pts[status==1]
        good_old = self.prev_points[status==1]
        # Re-init if too few tracks
        if len(good_new) < 50:
            pts = cv2.goodFeaturesToTrack(gray, mask=None, **self.feature_params)
            if pts is not None:
                self.prev_points = cv2.cornerSubPix(gray, pts, (10,10), (-1,-1), self.criteria_subpix)
            self.prev_gray = gray
            self.prev_stamp = stamp
            return

        # Compute displacement and velocities
        disp = np.median(good_new-good_old, axis=0)
        vel_px = disp/dt
        vel_mps = vel_px*self.pixel_to_meter

        # Publish
        out = Vector3Stamped(); out.header=msg.header
        out.vector.x, out.vector.y = float(vel_mps[0]), float(vel_mps[1])
        self.velocity_pub.publish(out)

        # Smooth
        self.velocity_buffer.append(vel_mps)
        sm = np.mean(self.velocity_buffer, axis=0)
        s_msg=Vector3Stamped(); s_msg.header=msg.header
        s_msg.vector.x, s_msg.vector.y = float(sm[0]), float(sm[1])
        self.velocity_smooth_pub.publish(s_msg)

        # Update state (always valid)
        self.prev_gray = gray.copy()
        self.prev_points = good_new.reshape(-1,1,2).astype(np.float32)
        self.prev_stamp = stamp

        # CSV log
        if self.writeCsv:
            elapsed = time.perf_counter()-start_time
            with open(self.csv_filename,'a',newline='') as f:
                csv.writer(f).writerow([time.time(), elapsed])
        if self.writeCsv:
            elapsed = time.perf_counter()-start_time
            with open(self.csv_filename,'a',newline='') as f:
                csv.writer(f).writerow([time.time(), elapsed])

# Note: Apply similar CSV logic to LiteFlowNet3 node if needed


def main(args=None):
    rclpy.init(args=args)
    node = LucasKanadeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    main()
