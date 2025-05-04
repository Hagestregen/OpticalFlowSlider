import cv2
import matplotlib.pyplot as plt
from torchvision.transforms import functional as F
from torchvision.models.optical_flow import raft_large
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
    frame1_float = frame1.astype(np.float32) / 255.0
    frame2_float = frame2.astype(np.float32) / 255.0
    mean_intensity_frame1 = np.mean(frame1_float)
    mean_intensity_frame2 = np.mean(frame2_float)
    if mean_intensity_frame2 > 0:
        frame2_float = frame2_float * (mean_intensity_frame1 / mean_intensity_frame2)
    frame2_float = np.clip(frame2_float, 0, 1)
    frame1_tensor = F.to_tensor(frame1_float).unsqueeze(0).to(device)
    frame2_tensor = F.to_tensor(frame2_float).unsqueeze(0).to(device)
    with torch.no_grad():
        flow = model(frame1_tensor, frame2_tensor)[0]
    flow = flow[0].permute(1, 2, 0).cpu().numpy()
    return flow, frame1_float, frame2_float

def convert_flow_to_meters(flow, conversion_factor=0.0009):
    flow_meters = flow * conversion_factor
    return flow_meters

def visualize_frames_with_title(frame1, frame2, title):
    frame1_rgb = cv2.cvtColor(frame1, cv2.COLOR_BGR2RGB)
    frame2_rgb = cv2.cvtColor(frame2, cv2.COLOR_BGR2RGB)
    fig, axes = plt.subplots(1, 2, figsize=(12, 6))
    axes[0].imshow(frame1_rgb)
    axes[0].set_title("First Frame")
    axes[0].axis("off")
    axes[1].imshow(frame2_rgb)
    axes[1].set_title("Last Frame")
    axes[1].axis("off")
    fig.suptitle(title, fontsize=14)
    plt.show()

def visualize_optical_flow_arrow(optical_flow, start_frame, scale, title="Arrow Head Optical Flow Visualization"):
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
    plt.title(title)
    plt.axis("off")
    plt.show()

# Main script
video_path = 'exposurevideo.avi'
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
model = raft_large(weights="C_T_SKHT_V2", progress=True).eval().to(device)

frame_rate, frame_count, frames = extract_frames(video_path)
first_frame = frames[0]
last_frame = frames[-1]

flow_pixels, norm_first_frame, norm_last_frame = compute_optical_flow(first_frame, last_frame, model, device)
flow_meters = convert_flow_to_meters(flow_pixels, conversion_factor=0.001063)
average_u_meters = np.mean(flow_meters[..., 0])
title = f"Optical Flow in X Direction (meters): {average_u_meters:.4f}"
visualize_frames_with_title(norm_first_frame, norm_last_frame, title)

# Visualize optical flow with arrows on the first frame
visualize_optical_flow_arrow(flow_meters, first_frame, scale=2500, title=f"Optical Flow (meters): {average_u_meters:.4f} Between First and Last Frame")