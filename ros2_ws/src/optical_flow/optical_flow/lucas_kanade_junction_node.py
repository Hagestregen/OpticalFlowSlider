#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud, Range, CameraInfo
from geometry_msgs.msg import Vector3Stamped
from cv_bridge import CvBridge
import cv2
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import message_filters
from collections import deque

class LucasKanadeNode(Node):
    def __init__(self):
        super().__init__('lucas_kanade_light_node')
        
        # Parameters
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.pixel_to_meter = 0.001063  # Updated via depth callback
        self.focal_length_x = None
        
        # Visualization parameters
        self.declare_parameter('show_live_feed', True)
        self.declare_parameter('show_flow', True)
        self.declare_parameter('show_mask', True)
        self.show_live_feed = self.get_parameter('show_live_feed').value
        self.show_flow = self.get_parameter('show_flow').value
        self.show_mask = self.get_parameter('show_mask').value
        
        # QoS profile
        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST
        )
        
        # Publishers
        self.velocity_pub = self.create_publisher(Vector3Stamped, '/optical_flow/LK_velocity', qos_profile)
        self.velocity_smooth_pub = self.create_publisher(Vector3Stamped, '/optical_flow/LK_smooth_velocity', qos_profile)
        self.live_feed_pub = self.create_publisher(Image, '/optical_flow/image_live_feed', 10)
        self.image_flow_pub = self.create_publisher(Image, '/optical_flow/image_flow', 10)
        self.image_mask_pub = self.create_publisher(Image, '/optical_flow/image_mask', 10)
        
        # Subscriptions
        self.image_sub = message_filters.Subscriber(self, Image, '/camera/camera/color/image_raw')
        self.junction_sub = message_filters.Subscriber(self, PointCloud, '/junction_detector/junctions')
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.junction_sub], queue_size=10, slop=0.01)
        self.ts.registerCallback(self.synced_callback)
        
        self.sub_depth = self.create_subscription(Range, '/camera/depth/median_distance', self.depth_callback, 10)
        self.sub_camera_info = self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.camera_info_callback, 10)
        
        # State variables
        self.bridge = CvBridge()
        self.prev_gray = None
        self.prev_points = None
        self.prev_mask = None
        self.prev_stamp = None
        self.velocity_buffer = deque(maxlen=5)  # For smoothed velocity
        
        # Feature detection and tracking parameters
        self.criteria_subpix = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 0.001)
        self.feature_params = dict(maxCorners=500, qualityLevel=0.3, minDistance=10, blockSize=7)
        self.lk_params = dict(winSize=(31, 31), maxLevel=3, criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 50, 0.001))
        self.displacement_threshold = 5.0  # Pixels, to filter out large movements
        
        self.get_logger().info('Lucas-Kanade Optical Flow Node with Junctions has been started.')
    
    def camera_info_callback(self, msg: CameraInfo):
        if self.focal_length_x is None:
            self.focal_length_x = msg.k[0]
            self.get_logger().info(f"Received focal length fx = {self.focal_length_x:.2f} px")
    
    def depth_callback(self, msg: Range):
        self.median_depth = float(msg.range)
        if self.focal_length_x:
            self.pixel_to_meter = self.median_depth / self.focal_length_x
    
    def synced_callback(self, image_msg, junction_msg):
        # Convert image to BGR and grayscale
        cv_bgr = self.bridge.imgmsg_to_cv2(image_msg, 'bgr8')
        gray = cv2.cvtColor(cv_bgr, cv2.COLOR_BGR2GRAY)
        
        # Create mask from junction points
        mask = np.zeros((self.height, self.width), dtype=np.uint8)
        junctions = [[p.x, p.y] for p in junction_msg.points]
        for p in junctions:
            x, y = int(p[0]), int(p[1])
            if 0 <= x < self.width and 0 <= y < self.height:
                cv2.circle(mask, (x, y), 5, 255, -1)
        # Dilation removed to prevent mask from including fish
        
        current_stamp = image_msg.header.stamp.sec + image_msg.header.stamp.nanosec * 1e-9
        
        # Initialize on first frame
        if self.prev_gray is None:
            self.prev_gray = gray.copy()
            self.prev_mask = mask.copy()
            self.prev_points = cv2.goodFeaturesToTrack(gray, mask=mask, **self.feature_params)
            if self.prev_points is not None:
                self.prev_points = cv2.cornerSubPix(gray, self.prev_points, (10, 10), (-1, -1), self.criteria_subpix)
            self.prev_stamp = current_stamp
            return
        
        # Track features
        next_points, status, error = cv2.calcOpticalFlowPyrLK(self.prev_gray, gray, self.prev_points, None, **self.lk_params)
        if next_points is None or status is None:
            self.reset_state(gray, mask, current_stamp)
            return
        
        # Select good points
        good_new = next_points[status == 1]
        good_old = self.prev_points[status == 1]
        
        # Filter points within both previous and current masks, with boundary checks
        selected_indices = [
            i for i, (p_old, p_new) in enumerate(zip(good_old, good_new))
            if (0 <= p_old[0] < self.width and 0 <= p_old[1] < self.height and
                0 <= p_new[0] < self.width and 0 <= p_new[1] < self.height and
                self.prev_mask[int(p_old[1]), int(p_old[0])] == 255 and
                mask[int(p_new[1]), int(p_new[0])] == 255)
        ]
        
        # Compute velocity with displacement threshold
        if len(selected_indices) > 0:
            selected_new = good_new[selected_indices]
            selected_old = good_old[selected_indices]
            dt = current_stamp - self.prev_stamp if current_stamp > self.prev_stamp else 1e-3
            displacements = selected_new - selected_old
            norms = np.linalg.norm(displacements, axis=1)
            valid = norms < self.displacement_threshold
            self.get_logger().info(f"Number of valid points: {np.sum(valid)}")
            if np.sum(valid) > 0:
                velocity_pixels = np.median(displacements[valid], axis=0) / dt
                velocity_mps = velocity_pixels * self.pixel_to_meter
            else:
                velocity_mps = np.array([0.0, 0.0])
        else:
            velocity_mps = np.array([0.0, 0.0])
            self.get_logger().warn("No tracked points within junction masks; setting velocity to zero.")
        
        # Publish raw velocity
        vel_msg = Vector3Stamped()
        vel_msg.header = image_msg.header
        vel_msg.vector.x = float(velocity_mps[0])
        vel_msg.vector.y = float(velocity_mps[1])
        vel_msg.vector.z = 0.0
        self.velocity_pub.publish(vel_msg)
        
        # Compute and publish smoothed velocity
        self.velocity_buffer.append(velocity_mps)
        if len(self.velocity_buffer) > 0:
            smoothed_velocity = np.mean(self.velocity_buffer, axis=0)
        else:
            smoothed_velocity = np.array([0.0, 0.0])
        vel_smooth_msg = Vector3Stamped()
        vel_smooth_msg.header = image_msg.header
        vel_smooth_msg.vector.x = float(smoothed_velocity[0])
        vel_smooth_msg.vector.y = float(smoothed_velocity[1])
        vel_smooth_msg.vector.z = 0.0
        self.velocity_smooth_pub.publish(vel_smooth_msg)
        
        # Visualize and publish images
        if self.show_live_feed:
            image_with_junctions = cv_bgr.copy()
            for p in junctions:
                cv2.circle(image_with_junctions, (int(p[0]), int(p[1])), 3, (0, 255, 0), -1)
            ros_live_feed = self.bridge.cv2_to_imgmsg(image_with_junctions, "bgr8")
            ros_live_feed.header = image_msg.header
            self.live_feed_pub.publish(ros_live_feed)
        
        if self.show_flow and len(selected_indices) > 0 and np.sum(valid) > 0:
            flow_vis = cv_bgr.copy()
            for new, old in zip(selected_new[valid], selected_old[valid]):
                a, b = new.ravel().astype(int)
                c, d = old.ravel().astype(int)
                cv2.line(flow_vis, (c, d), (a, b), (0, 255, 0), 2)
                cv2.circle(flow_vis, (a, b), 5, (0, 0, 255), -1)
            ros_flow = self.bridge.cv2_to_imgmsg(flow_vis, "bgr8")
            ros_flow.header = image_msg.header
            self.image_flow_pub.publish(ros_flow)
        
        if self.show_mask:
            mask_vis = np.zeros_like(cv_bgr)
            mask_vis[mask == 255] = [0, 255, 0]
            blended = cv2.addWeighted(cv_bgr, 0.7, mask_vis, 0.3, 0)
            ros_mask = self.bridge.cv2_to_imgmsg(blended, "bgr8")
            ros_mask.header = image_msg.header
            self.image_mask_pub.publish(ros_mask)
        
        # Update state with redetected features
        self.prev_gray = gray.copy()
        self.prev_mask = mask.copy()
        self.prev_points = cv2.goodFeaturesToTrack(gray, mask=mask, **self.feature_params)
        if self.prev_points is not None:
            self.prev_points = cv2.cornerSubPix(gray, self.prev_points, (10, 10), (-1, -1), self.criteria_subpix)
        self.prev_stamp = current_stamp
    
    def reset_state(self, gray, mask, current_stamp):
        self.prev_gray = gray.copy()
        self.prev_mask = mask.copy()
        self.prev_points = cv2.goodFeaturesToTrack(gray, mask=mask, **self.feature_params)
        if self.prev_points is not None:
            self.prev_points = cv2.cornerSubPix(gray, self.prev_points, (10, 10), (-1, -1), self.criteria_subpix)
        self.prev_stamp = current_stamp

def main(args=None):
    rclpy.init(args=args)
    node = LucasKanadeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()