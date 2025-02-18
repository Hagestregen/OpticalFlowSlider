#!/usr/bin/env python3
import pyrealsense2 as rs
import cv2
import numpy as np

def main():
    # Configure the RealSense pipeline to stream color and depth.
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    # Start the pipeline.
    pipeline.start(config)
    try:
        # Allow the camera to warm up.
        for _ in range(30):
            pipeline.wait_for_frames()
        
        # Get the latest frames.
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            print("Error: Could not acquire both depth and color frames.")
            return

        # Get the color intrinsics (if needed).
        color_profile = color_frame.profile.as_video_stream_profile()
        color_intrinsics = color_profile.intrinsics
        print(f"Color camera intrinsics:\n  fx = {color_intrinsics.fx}\n  fy = {color_intrinsics.fy}\n  width = {color_intrinsics.width}\n  height = {color_intrinsics.height}")

        # Get depth intrinsics.
        depth_profile = depth_frame.profile.as_video_stream_profile()
        depth_intrinsics = depth_profile.intrinsics
        print(f"Depth camera intrinsics:\n  fx = {depth_intrinsics.fx}\n  fy = {depth_intrinsics.fy}\n  width = {depth_intrinsics.width}\n  height = {depth_intrinsics.height}")

        # For this example, use the center of the depth image as the reference point.
        center_x = int(depth_intrinsics.width / 2)
        center_y = int(depth_intrinsics.height / 2)
        depth_value = depth_frame.get_distance(center_x, center_y)
        print(f"Depth at center ({center_x}, {center_y}): {depth_value:.3f} meters")

        # Compute the conversion factor (meters per pixel) using the horizontal focal length of the depth stream.
        conversion_factor = depth_value / depth_intrinsics.fx
        print(f"Conversion factor (meters per pixel): {conversion_factor:.6f}")

        cv2.destroyAllWindows()
    finally:
        pipeline.stop()

if __name__ == "__main__":
    main()
