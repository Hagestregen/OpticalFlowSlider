import cv2
import pyrealsense2 as rs
import numpy as np

# Start streaming and set up alignment
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

align_to = rs.stream.color
align = rs.align(align_to)

try:
    while True:
        # Wait for frames and align them
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        aligned_depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        if not aligned_depth_frame or not color_frame:
            continue

        # Retrieve the intrinsics from the depth frame
        depth_intrinsics = aligned_depth_frame.profile.as_video_stream_profile().intrinsics
        fx = depth_intrinsics.fx
        fy = depth_intrinsics.fy
        ppx = depth_intrinsics.ppx
        ppy = depth_intrinsics.ppy

        # Example pixel coordinate
        u, v = 400, 300
        # Get the depth at the pixel in meters
        z = aligned_depth_frame.get_distance(u, v)

        # Calculate the meter per pixel conversion factors at this depth
        meter_per_pixel_x = z / fx
        meter_per_pixel_y = z / fy

        print("At depth {:.2f} m, one pixel corresponds to {:.4f} m in x and {:.4f} m in y".format(
            z, meter_per_pixel_x, meter_per_pixel_y))

        # Optional: Visualize the point on the image
        color_image = np.asanyarray(color_frame.get_data())
        cv2.circle(color_image, (u, v), 4, (0, 0, 255), -1)
        cv2.imshow('Color Frame', color_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
