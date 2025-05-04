import pyrealsense2 as rs
import numpy as np
import cv2

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

# Find the RGB sensor
found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        rgb_sensor = s  # Store the RGB sensor
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)

# Enable auto-exposure initially to allow the camera to adjust
if found_rgb:
    rgb_sensor.set_option(rs.option.enable_auto_exposure, 1)

# Enable color stream
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming with the configured settings
pipeline.start(config)

# Wait for a few frames to allow auto-exposure to adjust (e.g., 30 frames)
print("Staring exposure")
for _ in range(30):
    frames = pipeline.wait_for_frames()


# Read current exposure and gain values after adjustment
current_exposure = rgb_sensor.get_option(rs.option.exposure)
current_gain = rgb_sensor.get_option(rs.option.gain)

# Disable auto-exposure and set manual exposure and gain
rgb_sensor.set_option(rs.option.enable_auto_exposure, 0)
rgb_sensor.set_option(rs.option.exposure, current_exposure)
rgb_sensor.set_option(rs.option.gain, current_gain)

# Video writer setup
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('exposurevideo.avi', fourcc, 30.0, (640, 480))

frame_count = 0
print("starting record")
try:
    while True:
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        # Convert images to numpy arrays
        color_image = np.asanyarray(color_frame.get_data())

        # Write the frame to the video file
        out.write(color_image)

        # Increment frame counter and check if 5 seconds (150 frames) have been recorded
        frame_count += 1
        if frame_count >= 150:
            break

        # Display the stream (optional)
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', color_image)
        cv2.waitKey(1)

    # Release the video writer after recording
    out.release()

finally:
    # Stop streaming
    pipeline.stop()