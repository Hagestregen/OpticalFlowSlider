import cv2
import matplotlib.pyplot as plt
from torchvision.transforms import functional as F
from torchvision.models.optical_flow import raft_large
from torchvision.utils import flow_to_image
from matplotlib.colors import hsv_to_rgb
import torch
import numpy as np


def extract_frames(video_path):
    cap = cv2.VideoCapture(video_path)
    frame_rate = int(cap.get(cv2.CAP_PROP_FPS))
    frames = []
    success = True
    while success:
        success, frame = cap.read()
        if not success:
            break
        frames.append(frame)
    cap.release()
    return frame_rate, len(frames), frames


def compute_optical_flow(frame1, frame2, model, device):
    frame1_tensor = F.to_tensor(frame1).unsqueeze(0).to(device)
    frame2_tensor = F.to_tensor(frame2).unsqueeze(0).to(device)
    with torch.no_grad():
        flow = model(frame1_tensor, frame2_tensor)[0]
    flow = flow[0].permute(1, 2, 0).cpu().numpy()
    return flow


def visualize_optical_flow_arrow(optical_flow, start_frame, scale):
    flow = np.array(optical_flow)
    vis_frame_with_arrows = start_frame.copy()
    H, W, _ = flow.shape
    for y in range(0, H, 100):
        for x in range(0, W, 100):
            dx, dy = flow[y, x]
            start_point = (x, y)
            end_point = (int(x + scale * dx), int(y + scale * dy))
            cv2.arrowedLine(vis_frame_with_arrows, start_point, end_point, (0, 255, 0), thickness=2, tipLength=0.3)
    vis_frame_with_arrows_rgb = cv2.cvtColor(vis_frame_with_arrows, cv2.COLOR_BGR2RGB)
    plt.figure(figsize=(6, 5))
    plt.imshow(vis_frame_with_arrows_rgb)
    plt.title("Arrow Head Optical Flow Visualization (Sampled Pixels)")
    plt.axis("off")


def visualize_optical_flow_grayscale(optical_flow, start_frame):
    # Compute the magnitude of the optical flow vectors
    flow_magnitude = np.sqrt(optical_flow[..., 0]**2 + optical_flow[..., 1]**2)  # Shape: [H, W]

    # Normalize the magnitude to the range [0, 255] for visualization
    flow_magnitude_normalized = cv2.normalize(flow_magnitude, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)

    # Convert the start frame to RGB for visualization alongside the flow
    start_frame_rgb = cv2.cvtColor(start_frame, cv2.COLOR_BGR2RGB)

    # Plot the grayscale intensity map and the original frame
    plt.figure(figsize=(12, 5))

    # Plot the original frame
    plt.subplot(1, 2, 1)
    plt.imshow(start_frame_rgb)
    plt.title("Original Frame")
    plt.axis("off")

    # Plot the grayscale intensity map
    plt.subplot(1, 2, 2)
    plt.imshow(flow_magnitude_normalized, cmap="gray")
    plt.title("Optical Flow (Grayscale Intensity Map)")
    plt.axis("off")


    plt.tight_layout()
    plt.show()


def plot_optical_flow_colorwheel():
    """
    Plots the colorwheel used in torchvision's flow_to_img function for visualizing optical flows.
    """
    # Set up the grid for flow visualization
    grid_size = 500  
    x, y = np.meshgrid(np.linspace(-1, 1, grid_size), np.linspace(-1, 1, grid_size))
    magnitude = np.sqrt(x**2 + y**2)
    angle = np.arctan2(-y, -x)  # Invert y-axis for correct orientation
    
    # Normalize magnitude to be within [0, 1]
    magnitude = np.clip(magnitude, 0, 1)

    # Convert to HSV
    hue = (angle + np.pi) / (2 * np.pi)  # Map angle to [0, 1]
    saturation = np.ones_like(hue)       # Maximum saturation
    value = magnitude                    # Value corresponds to magnitude
    
    hsv_image = np.stack((hue, saturation, value), axis=-1)
    rgb_image = hsv_to_rgb(hsv_image)
    
    return rgb_image

def visualize_optical_flow_with_colorwheel(optical_flow, first_frame, last_frame):
    # Generate the optical flow visualization
    flow = torch.tensor(optical_flow).permute(2, 0, 1)  # Shape: [2, H, W]
    flow_image = flow_to_image(flow)  # Shape: [3, H, W], values in [0, 255]
    vis_frame_with_color_flow = flow_image.permute(1, 2, 0).cpu().numpy()

    # Generate the color wheel
    color_wheel = plot_optical_flow_colorwheel()

    # Plot the color wheel and the optical flow visualization side by side
    plt.figure(figsize=(12, 6))

    # Color wheel plot
    plt.subplot(1, 2, 1)
    plt.imshow(color_wheel, extent=(-1, 1, -1, 1))
    plt.title("Optical Flow Colorwheel")
    plt.xlabel("Horizontal Flow (x)")
    plt.ylabel("Vertical Flow (y)")
    plt.axis('off')

    # Optical flow visualization plot
    plt.subplot(1, 2, 2)
    plt.imshow(vis_frame_with_color_flow)
    plt.title("Color-Coded Optical Flow Visualization")
    plt.axis("off")

    plt.tight_layout()
    plt.show()


# Main script
video_path = 'output.avi'
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
model = raft_large(weights="C_T_SKHT_V2", progress=True).eval().to(device)

frame_rate, frame_count, frames = extract_frames(video_path)
first_frame = frames[0]
last_frame = frames[-1]  # Using second frame as "last_frame" per original code

first_frame_rgb = cv2.cvtColor(first_frame, cv2.COLOR_BGR2RGB)
last_frame_rgb = cv2.cvtColor(last_frame, cv2.COLOR_BGR2RGB)

flow = compute_optical_flow(first_frame, last_frame, model, device)
visualize_optical_flow_arrow(flow, first_frame, scale=1)
visualize_optical_flow_arrow(flow, last_frame, scale=1)
# visualize_optical_flow_grayscale(flow, first_frame)
# visualize_optical_flow_with_colorwheel(flow, first_frame, last_frame)

# Create side-by-side plot
plt.figure(figsize=(6, 5))
plt.subplot(1, 2, 1)
plt.imshow(first_frame_rgb)
plt.title("First Frame")
plt.axis("off")
plt.subplot(1, 2, 2)
plt.imshow(last_frame_rgb)
plt.title("Second Frame")
plt.axis("off")
plt.tight_layout()

# Display both plots
plt.show()