#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
import cv2
import numpy as np
import torch
import torchvision.transforms.functional as F
from PIL import Image as PILImage
import time
import threading
import queue
from torch.amp import autocast
from torchvision.models.optical_flow import raft_small, Raft_Small_Weights
from torchvision.utils import flow_to_image
from geometry_msgs.msg import Vector3Stamped

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
    padded_img = F.pad(img, padding)
    return padded_img, padding

class RaftOpticalFlowNode(Node):
    def __init__(self):
        super().__init__('raft_optical_flow_node')

        # Subscription to RealSense camera topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10
        )

        # Publisher for velocity
        self.velocity_publisher = self.create_publisher(Vector3Stamped, '/optical_flow/raft_velocity', 10)

        # Initialize storage
        self.prev_image = None
        self.prev_time = None

        # Queues for parallel processing
        self.frame_queue = queue.Queue(maxsize=2)
        self.result_queue = queue.Queue()
        self.running = True

        # Load RAFT-Small model
        self.get_logger().info("Loading RAFT (small) model...")
        weights = Raft_Small_Weights.DEFAULT
        self.transforms = weights.transforms()
        self.model = raft_small(weights=weights, progress=False)
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.model.to(self.device).eval()
        self.get_logger().info(f"RAFT Optical Flow Node started on {self.device}")

        # Conversion factor: pixels -> meters
        self.conv_factor = 0.000566

        # Start worker thread for inference
        self.worker = threading.Thread(target=self.process_flow, daemon=True)
        self.worker.start()

    def image_callback(self, msg):
        start_total = time.time()

        # Convert ROS2 Image message to NumPy array
        try:
            cv_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            if msg.encoding == 'bgr8':
                cv_image = cv_image  # Already in BGR
            elif msg.encoding == 'rgb8':
                cv_image = cv_image[..., [2, 1, 0]]  # RGB to BGR
            else:
                self.get_logger().error(f"Unsupported encoding: {msg.encoding}")
                return
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {str(e)}")
            return

        # Preprocess image
        cv_image = cv_image[:cv_image.shape[0] // 2, :, :]  # Crop to top half
        rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        current_image = PILImage.fromarray(np.uint8(rgb_image))
        current_time = time.time()

        if self.prev_time is None:
            self.prev_image = current_image
            self.prev_time = current_time
            return

        dt = current_time - self.prev_time
        if dt <= 0:
            dt = 1e-3
        self.prev_time = current_time

        # Queue frame for processing
        try:
            self.frame_queue.put_nowait((self.prev_image, current_image, dt))
            self.prev_image = current_image
        except queue.Full:
            pass  # Skip if queue is full

        # Publish result if available
        try:
            velocity = self.result_queue.get_nowait()
            
            vel_msg = Vector3Stamped()
            vel_msg.header = msg.header  # Use the same header from the image
            vel_msg.vector.x = float(velocity)
            vel_msg.vector.y = 0.0  # assuming 2D motion
            vel_msg.vector.z = 0.0  # assuming 2D motion
            # velocity_msg = Float64()
            # velocity_msg.data = float(velocity)
            self.velocity_publisher.publish(vel_msg)
            elapsed = time.time() - start_total
            # self.get_logger().info(f"Published at {1/elapsed:.1f} Hz, prep time: {elapsed:.3f}s")
        except queue.Empty:
            pass

    def process_flow(self):
        while self.running:
            try:
                prev_image, current_image, dt = self.frame_queue.get(timeout=1.0)
                start_process = time.time()

                # Transform images
                img1, img2 = self.transforms(prev_image, current_image)
                img1 = img1.unsqueeze(0)
                img2 = img2.unsqueeze(0)
                img1, _ = pad_to_multiple(img1, multiple=8)
                img2, _ = pad_to_multiple(img2, multiple=8)

                # Compute optical flow with FP16
                start_inference = time.time()
                with torch.no_grad():
                    with autocast('cuda'):
                        flows = self.model(img1.to(self.device), img2.to(self.device))
                flow = flows[-1][0].cpu()
                inference_time = time.time() - start_inference

                # Compute velocity
                velocity_mps = (flow / dt) * self.conv_factor
                avg_velocity = velocity_mps.mean(dim=(1, 2))[0]
                self.result_queue.put(float(avg_velocity))

                total_process = time.time() - start_process
                # self.get_logger().info(f"Inference: {inference_time:.3f}s, Process: {total_process:.3f}s")
            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f"Error in process_flow: {str(e)}")
                continue

    def destroy_node(self):
        self.running = False
        self.worker.join(timeout=2.0)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RaftOpticalFlowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()