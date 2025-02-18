#!/usr/bin/env python3
import pyrealsense2 as rs
import cv2
import numpy as np

def main():
    # Configure the RealSense pipeline to stream the color image.
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    
    # Start the pipeline.
    pipeline.start(config)
    
    try:
        # Allow the camera some time to warm up.
        for _ in range(30):
            pipeline.wait_for_frames()
        
        # Get a color frame.
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            print("Error: Could not acquire a color frame.")
            return

        # Convert the color frame to an OpenCV image.
        color_image = np.asanyarray(color_frame.get_data())
        
        # Show the image and let the user select an ROI for the reference object.
        cv2.imshow("Color Image", color_image)
        print("Select the ROI for the object with known dimensions and press ENTER or SPACE.")
        roi = cv2.selectROI("Color Image", color_image, fromCenter=False, showCrosshair=True)
        cv2.destroyWindow("Color Image")
        
        x, y, w, h = roi
        if w == 0 or h == 0:
            print("No valid ROI was selected. Exiting.")
            return

        print(f"Selected ROI dimensions: {w} pixels wide, {h} pixels high.")

        # Ask the user for the physical dimensions of the object in meters.
        known_width_str = input("Enter the known physical width (in meters) of the object: ")
        known_height_str = input("Enter the known physical height (in meters) of the object: ")
        try:
            known_width = float(known_width_str)
            known_height = float(known_height_str)
        except ValueError:
            print("Invalid input. Please enter numeric values.")
            return
        
        # Calculate conversion factors.
        conv_factor_width = known_width / w  # meters per pixel (width)
        conv_factor_height = known_height / h  # meters per pixel (height)

        print(f"Conversion factor (width): {conv_factor_width:.6f} m/px")
        print(f"Conversion factor (height): {conv_factor_height:.6f} m/px")

        # Optionally, compute an average conversion factor.
        avg_conv_factor = (conv_factor_width + conv_factor_height) / 2.0
        print(f"Average conversion factor: {avg_conv_factor:.6f} m/px")
        
    finally:
        pipeline.stop()

if __name__ == "__main__":
    main()
