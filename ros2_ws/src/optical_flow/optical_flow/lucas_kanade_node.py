#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3Stamped
import cv2
import numpy as np
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import os
import csv
import time
from collections import deque

class LucasKanadeNode(Node):
    def __init__(self):
        super().__init__('lucas_kanade_node')
        
        # --- Parameters ---
        self.width = 640
        self.height = 480
        self.fps = 30
        self.pixel_to_meter = 0.001063  # placeholder; update with actual depth data

        # Error & track thresholds
        self.err_thresh    = 4.0   # allow higher solver residual
        self.fb_err_thresh = 6.0   # allow larger forward-backward error
        self.min_tracks    = 20    # minimum survivors before reseeding

        # Shi-Tomasi & LK params
        self.feature_params = dict(
            maxCorners=300,
            qualityLevel=0.01,
            minDistance=10,
            blockSize=7
        )
        self.lk_params = dict(
            winSize=(15, 15),
            maxLevel=3,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 20, 0.01)
        )
        self.criteria_subpix = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 20, 0.0001)

        # CSV logging setup
        self.csv_filename = f"lk_optimized_{self.width}x{self.height}.csv"
        self.csv_file     = open(self.csv_filename, 'a', newline='')
        self.csv_writer   = csv.writer(self.csv_file)
        if os.path.getsize(self.csv_filename) == 0:
            self.csv_writer.writerow(['timestamp', 'inference_time_s'])
        self._last_flush = time.time()

        # QoS for image transport
        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST
        )

        # ROS subscriptions and publishers
        self.subscription        = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            qos
        )
        self.velocity_pub        = self.create_publisher(
            Vector3Stamped,
            '/optical_flow/LK_velocity',
            qos
        )
        self.velocity_smooth_pub = self.create_publisher(
            Vector3Stamped,
            '/optical_flow/LK_smooth_velocity',
            qos
        )

        # Bridge & CLAHE
        self.bridge = CvBridge()
        self.clahe  = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))

        # State
        self.prev_gray   = None
        self.prev_points = None
        self.prev_stamp  = None
        self.velocity_buffer = deque(maxlen=5)

        self.get_logger().info('Lucas–Kanade Optical Flow Node started.')

    def _init_features(self, gray, stamp):
        # Full reinitialization of feature set
        self.prev_gray = gray
        pts = cv2.goodFeaturesToTrack(gray, mask=None, **self.feature_params)
        if pts is not None:
            self.prev_points = cv2.cornerSubPix(
                gray, pts, (10,10), (-1,-1), self.criteria_subpix
            )
        else:
            self.prev_points = None
        self.prev_stamp = stamp
        self.get_logger().warning('Full feature reinit')

    def _add_new_features(self, good_pts, gray):
        # Partial reseeding: add new features around surviving points
        mask = np.ones_like(gray, dtype=np.uint8) * 255
        for x, y in good_pts.astype(int):
            cv2.circle(mask, (x,y), self.feature_params['minDistance'], 0, -1)
        new_pts = cv2.goodFeaturesToTrack(gray, mask=mask, **self.feature_params)
        if new_pts is not None:
            new_pts = cv2.cornerSubPix(gray, new_pts, (10,10), (-1,-1), self.criteria_subpix)
            all_pts = np.vstack([good_pts, new_pts.reshape(-1,2)])
        else:
            all_pts = good_pts
        # Trim to maxCorners
        max_n = self.feature_params['maxCorners']
        if len(all_pts) > max_n:
            all_pts = all_pts[:max_n]
        return all_pts

    def image_callback(self, msg: Image):
        start = time.perf_counter()
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")
            return

        # Preprocess
        gray = cv2.GaussianBlur(
            cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY),
            (3,3), 0
        )
        gray = self.clahe.apply(gray)
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # First frame
        if self.prev_gray is None:
            pts = cv2.goodFeaturesToTrack(gray, mask=None, **self.feature_params)
            self.prev_points = pts
            self.prev_gray   = gray
            self.prev_stamp  = ts
            return
        # Time delta
        dt = max(ts - self.prev_stamp, 1e-3)

        # Forward LK
        p1, st1, err1 = cv2.calcOpticalFlowPyrLK(
            self.prev_gray, gray, self.prev_points, None, **self.lk_params
        )
        if p1 is None or st1 is None or err1 is None:
            self._init_features(gray, ts)
            return

        # Backward LK
        p0r, st2, err2 = cv2.calcOpticalFlowPyrLK(
            gray, self.prev_gray, p1, None, **self.lk_params
        )
        if p0r is None or st2 is None or err2 is None:
            self._init_features(gray, ts)
            return

        # Build mask of good tracks
        fb_err = np.linalg.norm(
            self.prev_points.reshape(-1,2) - p0r.reshape(-1,2),
            axis=1
        )
        mask = (
            (st1.ravel()==1) &
            (st2.ravel()==1) &
            (err1.ravel() < self.err_thresh) &
            (fb_err     < self.fb_err_thresh)
        )
        good_old = self.prev_points.reshape(-1,2)[mask]
        good_new = p1.reshape(-1,2)[mask]

        # Log survival
        self.get_logger().info(
            f"Tracks before filter: {len(self.prev_points)}, after: {len(good_new)}"
        )

        # # Partial reseed if needed
        # if len(good_new) < self.min_tracks:
        #     good_new = self._add_new_features(good_new, gray)
        #     self.get_logger().info(
        #         f"After partial reseed: {len(good_new)} tracks"
        #     )

        # Partial reseed
        if len(good_new) < self.min_tracks:
            good_new = self._add_new_features(good_new, gray)
            self.get_logger().info(f"After partial reseed: {len(good_new)} tracks")
            # Update state, skip velocity this frame
            self.prev_points = good_new.reshape(-1,1,2)
            self.prev_gray   = gray.copy()
            self.prev_stamp  = ts
            return

        # Now we can safely RANSAC, because good_old & good_new match
        if len(good_new) < 3:
            vel_px = np.zeros(2)
        else:
            M, inliers = cv2.estimateAffinePartial2D(
                good_old, good_new,
                method=cv2.RANSAC,
                ransacReprojThreshold=5.0
            )
            if M is not None:
                dx, dy = M[0,2], M[1,2]
                vel_px = np.array([dx, dy]) / dt
            else:
                vel_px = np.zeros(2)


        vel_mps = vel_px * self.pixel_to_meter

        # Publish raw
        vel_msg = Vector3Stamped()
        vel_msg.header = msg.header
        vel_msg.vector.x = float(vel_mps[0])
        vel_msg.vector.z = 0.0
        self.velocity_pub.publish(vel_msg)

        # Smooth
        self.velocity_buffer.append(vel_px)
        vel_smooth = np.mean(self.velocity_buffer, axis=0)
        vel_smooth_msg = Vector3Stamped()
        vel_smooth_msg.header = msg.header
        vel_smooth_msg.vector.x = float(vel_smooth[0])
        vel_smooth_msg.vector.z = 0.0
        self.velocity_smooth_pub.publish(vel_smooth_msg)

        # Update state
        self.prev_points = good_new.reshape(-1,1,2)
        self.prev_gray   = gray.copy()
        self.prev_stamp  = ts

        # Log timing
        elapsed = time.perf_counter() - start
        now = time.time()
        self.csv_writer.writerow([now, elapsed])
        if now - self._last_flush > 10.0:
            self.csv_file.flush()
            self._last_flush = now

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LucasKanadeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()