#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import torch
import torchvision.transforms.functional as F
from PIL import Image as PILImage
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3Stamped

# Import RAFT modules from torchvision for the small variant.
from torchvision.models.optical_flow import raft_small, Raft_Small_Weights
from torchvision.utils import flow_to_image

def pad_to_multiple(img, multiple=8):
    """
    Pads a tensor image so its height and width are divisible by 'multiple'.
    
    Args:
        img (torch.Tensor): Image tensor of shape [B, C, H, W].
        multiple (int): The value by which H and W should be divisible.
        
    Returns:
        padded_img (torch.Tensor): The padded image.
        padding (tuple): The padding applied (left, top, right, bottom).
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

        # Create a QoS profile (best effort, last history).
        qos_profile = rclpy.qos.QoSProfile(
            depth=10,
            reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST
        )

        # Subscribe to the color image topic.
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            qos_profile
        )

        self.velocity_publisher = self.create_publisher(Vector3Stamped, '/optical_flow/raft_velocity', qos_profile)
        self.bridge = CvBridge()

        self.prev_image = None   # Store the previous frame as a PIL image
        self.prev_time = None    # Store the previous frame's timestamp

        # Load the RAFT model (small variant).
        self.get_logger().info("Loading RAFT (small) model...")
        weights = Raft_Small_Weights.DEFAULT
        self.transforms = weights.transforms()  # Expects two PIL images => outputs [C, H, W] each
        self.model = raft_small(weights=weights, progress=False)
        
        # Set device to GPU if available.
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.model.to(self.device).eval()
        self.get_logger().info("RAFT Optical Flow Node (small) started on device: " + self.device)

        # Optional conversion factor: pixels -> meters. Adjust as needed.
        # Example: 0.000566 is for your particular setup.
        self.conv_factor = 0.000566  

    def image_callback(self, msg: Image):
        # Convert the ROS image to OpenCV BGR format.
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")
            return

        # # (Optional) Downsample to half resolution to speed up RAFT inference.
        # cv_image = cv2.resize(
        #     cv_image, 
        #     (cv_image.shape[1] // 2, cv_image.shape[0] // 2)
        # )

        # Convert from BGR to RGB, then to uint8 for PIL image.
        rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        rgb_image = np.uint8(rgb_image)
        current_image = PILImage.fromarray(rgb_image)

        # Get the current ROS timestamp in seconds.
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # # Print out the timestamps for debugging:
        # if self.prev_time is not None:
        #     self.get_logger().info(
        #         f"Current time: {current_time:.6f}, Prev time: {self.prev_time:.6f}, "
        #         f"Delta t: {(current_time - self.prev_time):.6f}"
        #     )

        # If this is the first frame, just store and return.
        if self.prev_time is None:
            self.prev_image = current_image
            self.prev_time = current_time
            return

        # Calculate the time difference (dt).
        dt = current_time - self.prev_time
        if dt <= 0:
            dt = 1e-3  # Avoid zero or negative dt
        self.prev_time = current_time

        # Apply the RAFT transforms to the previous and current frames (both PIL images).
        try:
            # Each transform call produces a [C, H, W] tensor.
            img1, img2 = self.transforms(self.prev_image, current_image)
        except Exception as e:
            self.get_logger().error(f"Error in RAFT transforms: {e}")
            self.prev_image = current_image
            return

        # Add a batch dimension => [1, 3, H, W].
        img1 = img1.unsqueeze(0)
        img2 = img2.unsqueeze(0)

        # Pad so H, W are multiples of 8.
        img1, _ = pad_to_multiple(img1, multiple=8)
        img2, _ = pad_to_multiple(img2, multiple=8)

        # Compute optical flow with RAFT (on GPU if available).
        with torch.no_grad():
            flows = self.model(img1.to(self.device), img2.to(self.device))

        # flows[-1] should be shape [1, 2, H, W]. Grab [2, H, W] from the batch dimension.
        flow = flows[-1][0].cpu()  # shape [2, H, W]

        # Convert flow (pixels/frame) to pixels/second.
        velocity_pixels = flow / dt

        # Convert to meters/second (if conv_factor is correct for your setup).
        velocity_mps = velocity_pixels * self.conv_factor

        # -------------------------------------------------------------
        # CROP: Use only the TOP HALF of the image for average velocity.
        # -------------------------------------------------------------
        _, height, width = velocity_mps.shape  # (2, H, W)
        top_half = velocity_mps[:, :height // 2, :]

        # Average velocity in the top half of the image:
        avg_velocity_top = top_half.mean(dim=(1, 2))  # shape [2]
        # dx, dy in top half
        # self.get_logger().info(
        #     f"Avg velocity (TOP HALF): dx: {avg_velocity_top[0]:.3f}, "
        #     f"dy: {avg_velocity_top[1]:.3f} m/s"
        # )

        vel_msg = Vector3Stamped()
        vel_msg.header = msg.header  # Use the same header from the image
        vel_msg.vector.x = float(avg_velocity_top[0])
        vel_msg.vector.y = 0.0  # assuming 2D motion
        vel_msg.vector.z = 0.0  # assuming 2D motion
        # velocity_msg = Float64()
        # velocity_msg.data = float(avg_velocity_top[0])
        self.velocity_publisher.publish(vel_msg)

        # If you still want the overall average across the entire frame:
        # avg_velocity_full = velocity_mps.mean(dim=(1, 2))
        # dx, dy in full frame
        # self.get_logger().info(
        #     f"Avg velocity (FULL): dx: {avg_velocity_full[0]:.3f}, "
        #     f"dy: {avg_velocity_full[1]:.3f} m/s"
        # )

        # (Optional) Visualize the flow to confirm motion is detected.
        # flow_img = flow_to_image(flow.unsqueeze(0))[0].numpy()  # [3, H, W] => [H, W, 3]
        # flow_img = flow_img.transpose(1, 2, 0)
        # flow_img = np.ascontiguousarray(flow_img, dtype=np.uint8)
        # cv2.imshow("RAFT Optical Flow (small)", flow_img)
        # cv2.waitKey(1)

        # Update the previous image.
        self.prev_image = current_image

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
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
