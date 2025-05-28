#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Range, CameraInfo
from geometry_msgs.msg import Vector3Stamped
from cv_bridge import CvBridge
import numpy as np
import torch
from torch.amp import autocast
from torchvision.transforms.functional import pad
from torchvision.models.optical_flow import raft_small, Raft_Small_Weights, raft_large, Raft_Large_Weights
from PIL import Image as PILImage
import os
import csv
import time
from collections import deque
import cv2

def pad_to_multiple(img, multiple=8):
    """
    Pads a tensor image so its height and width are divisible by 'multiple'.
    """
    _, _, H, W = img.shape
    pad_h = (multiple - (H % multiple)) % multiple
    pad_w = (multiple - (W % multiple)) % multiple
    pad_top = pad_h // 2
    pad_bottom = pad_h - pad_top
    pad_left = pad_w // 2
    pad_right = pad_w - pad_left
    padding = (pad_left, pad_top, pad_right, pad_bottom)
    return pad(img, padding), padding

class RaftOpticalFlowNode(Node):
    def __init__(self):
        super().__init__('raft_large_node')
        
        small = False  # Set to True for RAFT-small, False for RAFT-large
        self.writeCsv = False
        self.focal_length_x = None
        
        # Frame settings
        # self.width = 640
        # self.height = 480
        # Declare parameters for width and height with default values
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        
        # Get the parameter values
        self.width = self.get_parameter('width').get_parameter_value().integer_value
        self.height = self.get_parameter('height').get_parameter_value().integer_value
        self.width_depth = 640
        self.height_depth = 480
        self.fps = 30
        self.pixel_to_meter = 0.000857  # Will be set after receiving camera info


        
        if self.writeCsv:
            # Prepare CSV file for inference times
            self.csv_filename = f"raft_large_{self.width}x{self.height}.csv"
            # Write header if new file
            if not os.path.isfile(self.csv_filename):
                with open(self.csv_filename, 'w', newline='') as csvfile:
                    writer = csv.writer(csvfile)
                    writer.writerow(['timestamp', 'inference_time_s'])
        
        # ROS interfaces
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.image_callback, 10)
        self.sub_depth = self.create_subscription(
            Range, '/camera/depth/median_distance', self.depth_callback, 10)
        self.sub_camera_info = self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info', self.camera_info_callback, 10)

        # Publishers for raw & smoothed velocity
        if small:
            self.raw_pub = self.create_publisher(Vector3Stamped, '/optical_flow/raft_small_velocity', 10)
            self.smooth_pub = self.create_publisher(Vector3Stamped, '/optical_flow/raft_small_smooth_velocity', 10)
        else:
            self.raw_pub = self.create_publisher(Vector3Stamped, '/optical_flow/raft_large_velocity', 10)
            self.smooth_pub = self.create_publisher(Vector3Stamped, '/optical_flow/raft_large_smooth_velocity', 10)

        # State for image-based flow
        self.prev_image = None
        self.prev_time = None

        # Buffer for smoothing
        self.velocity_buffer = deque(maxlen=3)

        # Load RAFT model
        self.get_logger().info("Loading RAFT model…")
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        print(f"Using device: {self.device}")
        if small:
            weights = Raft_Small_Weights.DEFAULT
            self.get_logger().info("Using RAFT-small model")
            self.model = raft_small(weights=weights).to(self.device).eval()
        else:
            weights = Raft_Large_Weights.DEFAULT
            self.get_logger().info("Using RAFT-large model")
            self.model = raft_large(weights=weights).to(self.device).eval()
        self.transforms = weights.transforms()
        # self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')



    def camera_info_callback(self, msg: CameraInfo):
        if self.focal_length_x is None:
            self.focal_length_x = msg.k[0]  # fx is the first element in the K matrix
            self.get_logger().info(f"Received focal length fx = {self.focal_length_x:.2f} px")

    def depth_callback(self, msg: Range):
        if self.focal_length_x is not None:
            self.pixel_to_meter = msg.range / self.focal_length_x
            self.get_logger().info(f"pixel_to_meter updated to {self.pixel_to_meter:.6f} m/px")
        else:
            self.get_logger().warn("Focal length not yet received; cannot update pixel_to_meter")

    def image_callback(self, msg: Image):
        if self.writeCsv:
            start_time = time.perf_counter()
        if self.focal_length_x is None or self.pixel_to_meter is None:
            self.get_logger().warn("Focal length not yet received; skipping image processing")
            return

        # Convert to BGR frame
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return

        # Compute dt from ROS header
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.prev_time is None:
            self.prev_image = frame
            self.prev_time = stamp
            return
        dt = stamp - self.prev_time
        if dt <= 0:
            dt = 1e-3
        self.prev_time = stamp

        # Prepare for inference
        img1 = PILImage.fromarray(cv2.cvtColor(self.prev_image, cv2.COLOR_BGR2RGB))
        img2 = PILImage.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        inp1, inp2 = self.transforms(img1, img2)
        inp1, _ = pad_to_multiple(inp1.unsqueeze(0))
        inp2, _ = pad_to_multiple(inp2.unsqueeze(0))

        # Inference
        with torch.no_grad(), autocast(self.device.type):
            flows = self.model(inp1.to(self.device), inp2.to(self.device))
        flow = flows[-1][0].cpu().numpy()

        # Compute mean horizontal velocity (px→m)
        vx = float((flow[0] / dt).mean() * self.pixel_to_meter)

        # Publish raw velocity
        raw_msg = Vector3Stamped()
        raw_msg.header = msg.header
        raw_msg.vector.x = vx
        raw_msg.vector.y = 0.0
        raw_msg.vector.z = 0.0
        self.raw_pub.publish(raw_msg)

        # Publish smoothed velocity
        self.velocity_buffer.append(vx)
        smooth_v = sum(self.velocity_buffer) / len(self.velocity_buffer)
        smooth_msg = Vector3Stamped()
        smooth_msg.header = msg.header
        smooth_msg.vector.x = smooth_v
        smooth_msg.vector.y = 0.0
        smooth_msg.vector.z = 0.0
        self.smooth_pub.publish(smooth_msg)

        # Update for next frame
        self.prev_image = frame
        
        if self.writeCsv:
                end_time = time.perf_counter()
                inference_time = end_time - start_time
                
                # Append to CSV
                timestamp = time.time()
                with open(self.csv_filename, 'a', newline='') as csvfile:
                    csv.writer(csvfile).writerow([timestamp, inference_time])

    def destroy_node(self):
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RaftOpticalFlowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()




# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image, Range
# from geometry_msgs.msg import Vector3Stamped
# from cv_bridge import CvBridge
# import numpy as np
# import torch
# from torch.amp import autocast
# from torchvision.transforms.functional import pad
# from torchvision.models.optical_flow import raft_small, Raft_Small_Weights, raft_large, Raft_Large_Weights
# from PIL import Image as PILImage
# import os
# import csv
# import time
# import pyrealsense2 as rs
# from collections import deque
# import cv2

# def pad_to_multiple(img, multiple=8):
#     """
#     Pads a tensor image so its height and width are divisible by 'multiple'.
#     """
#     _, _, H, W = img.shape
#     pad_h = (multiple - (H % multiple)) % multiple
#     pad_w = (multiple - (W % multiple)) % multiple
#     pad_top = pad_h // 2
#     pad_bottom = pad_h - pad_top
#     pad_left = pad_w // 2
#     pad_right = pad_w - pad_left
#     padding = (pad_left, pad_top, pad_right, pad_bottom)
#     return pad(img, padding), padding
# class RaftOpticalFlowNode(Node):
#     def __init__(self):
#         super().__init__('raft_optical_flow_node')
        
        
        
#         small = False
        
        
#         # Frame & depth settings
#         self.width = 640
#         self.height = 480
#         self.width_depth = 640
#         self.height_depth = 480
#         self.fps = 30
#         self.pixel_to_meter = 0.001063

#         # # CSV for inference timings
#         # self.csv_filename = f"raft_L_inference_{self.width}x{self.height}.csv"
#         # if not os.path.exists(self.csv_filename):
#         #     with open(self.csv_filename, 'w', newline='') as f:
#         #         writer = csv.writer(f)
#         #         writer.writerow(['timestamp', 'inference_time_s'])

#         # Read RealSense intrinsics once
#         self.get_logger().info('Reading RealSense intrinsics…')
#         pipeline = rs.pipeline()
#         cfg = rs.config()
#         cfg.enable_stream(rs.stream.color,  self.width, self.height, rs.format.rgb8, self.fps)
#         cfg.enable_stream(rs.stream.depth,  self.width_depth, self.height_depth, rs.format.z16,  self.fps)
#         profile = pipeline.start(cfg)
#         intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
#         self.focal_length_x = intr.fx
#         pipeline.stop()
#         self.get_logger().info(f"RealSense fx = {self.focal_length_x:.2f} px")

#         # ROS interfaces
#         self.bridge    = CvBridge()
#         self.subscription = self.create_subscription(
#             Image, '/camera/camera/color/image_raw', self.image_callback, 10)
#         self.sub_depth = self.create_subscription(
#             Range, '/camera/depth/median_distance', self.depth_callback, 10)

#         # Publishers for raw & smoothed velocity
#         if small:
#             self.raw_pub    = self.create_publisher(Vector3Stamped, '/optical_flow/raft_small_velocity',        10)
#             self.smooth_pub = self.create_publisher(Vector3Stamped, '/optical_flow/raft__small_smooth_velocity', 10)
#         else:
#             self.raw_pub    = self.create_publisher(Vector3Stamped, '/optical_flow/raft_large_velocity',        10)
#             self.smooth_pub = self.create_publisher(Vector3Stamped, '/optical_flow/raft_large_smooth_velocity', 10)

#         # State for image‐based flow
#         self.prev_image = None
#         self.prev_time  = None

#         # Buffer for smoothing
#         self.velocity_buffer = deque(maxlen=5)

#         # Load RAFT‐small model
#         self.get_logger().info("Loading RAFT‐small model…")
#         if small:
#             weights = Raft_Small_Weights.DEFAULT
#         else:
#             weights = Raft_Large_Weights.DEFAULT
#         self.transforms = weights.transforms()
#         self.device     = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
#         if small:
#             self.get_logger().info("Using RAFT‐small model")
#             self.model      = raft_small(weights=weights).to(self.device).eval()
#         else:
#             self.get_logger().info("Using RAFT‐large model")
#             self.model      = raft_large(weights=weights).to(self.device).eval()


#     def depth_callback(self, msg: Range):
#         self.pixel_to_meter = msg.range / self.focal_length_x
#         self.get_logger().info(f"pixel_to_meter updated to {self.pixel_to_meter:.6f} m/px")

#     def image_callback(self, msg: Image):
#         # Convert to BGR frame
#         try:
#             frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
#         except Exception as e:
#             self.get_logger().error(f"CV Bridge error: {e}")
#             return

#         # Compute dt from ROS header
#         stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
#         if self.prev_time is None:
#             self.prev_image = frame
#             self.prev_time  = stamp
#             return
#         dt = stamp - self.prev_time
#         if dt <= 0:
#             dt = 1e-3
#         self.prev_time = stamp

#         # Prepare for inference
#         # t0 = time.perf_counter()
#         img1 = PILImage.fromarray(cv2.cvtColor(self.prev_image, cv2.COLOR_BGR2RGB))
#         img2 = PILImage.fromarray(cv2.cvtColor(frame,           cv2.COLOR_BGR2RGB))
#         inp1, inp2 = self.transforms(img1, img2)
#         inp1, _ = pad_to_multiple(inp1.unsqueeze(0))
#         inp2, _ = pad_to_multiple(inp2.unsqueeze(0))

#         # Inference
#         with torch.no_grad(), autocast(self.device.type):
#             flows = self.model(inp1.to(self.device), inp2.to(self.device))
#         flow = flows[-1][0].cpu().numpy()

#         # Compute mean horizontal velocity (px→m)
#         vx = float((flow[0] / dt).mean() * self.pixel_to_meter)

#         # Publish raw velocity
#         raw_msg = Vector3Stamped()
#         raw_msg.header = msg.header
#         raw_msg.vector.x = vx
#         raw_msg.vector.y = 0.0
#         raw_msg.vector.z = 0.0
#         self.raw_pub.publish(raw_msg)

#         # Publish smoothed velocity
#         self.velocity_buffer.append(vx)
#         smooth_v = sum(self.velocity_buffer) / len(self.velocity_buffer)
#         smooth_msg = Vector3Stamped()
#         smooth_msg.header = msg.header
#         smooth_msg.vector.x = smooth_v
#         smooth_msg.vector.y = 0.0
#         smooth_msg.vector.z = 0.0
#         self.smooth_pub.publish(smooth_msg)

#         # # Log inference+publish time
#         # t1 = time.perf_counter()
#         # with open(self.csv_filename, 'a', newline='') as f:
#         #     csv.writer(f).writerow([t1, t1 - t0])

#         # Update for next frame
#         self.prev_image = frame

#     def destroy_node(self):
#         super().destroy_node()

# def main(args=None):
#     rclpy.init(args=args)
#     node = RaftOpticalFlowNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.get_logger().info("Shutting down.")
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()
