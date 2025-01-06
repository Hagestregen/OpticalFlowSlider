import pyrealsense2 as rs
import numpy as np
import cv2
import os
from datetime import datetime
import csv

class RealSenseRecorder:
    def __init__(self, video_dir='../../../Data/realsense/videos', imu_dir='../../../Data/realsense/imu_data', duration=5):
        self.video_dir = video_dir
        self.imu_dir = imu_dir
        self.duration = duration
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        # Enable only RGB and IMU streams
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 100)
        self.config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)
        self.video_writer = None
        self.imu_file = None

    def start_recording(self):
        # Create directories if they don't exist
        os.makedirs(self.video_dir, exist_ok=True)
        os.makedirs(self.imu_dir, exist_ok=True)

        # Generate file paths with timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        video_path = os.path.join(self.video_dir, f"{timestamp}.mp4")  # Save as MP4
        imu_path = os.path.join(self.imu_dir, f"{timestamp}.csv")

        # Initialize video writer and IMU file
        self.video_writer = cv2.VideoWriter(video_path, cv2.VideoWriter_fourcc(*'mp4v'), 30, (640, 480))  # Use MP4 codec
        self.imu_file = open(imu_path, 'w', newline='')
        imu_writer = csv.writer(self.imu_file)
        imu_writer.writerow(["timestamp", "accel_x", "accel_y", "accel_z", "gyro_x", "gyro_y", "gyro_z"])


        # Start the pipeline
        self.pipeline.start(self.config)
        start_time = datetime.now()

        try:
            while (datetime.now() - start_time).seconds < self.duration:
                frames = self.pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()
                accel_frame = frames.first_or_default(rs.stream.accel)
                gyro_frame = frames.first_or_default(rs.stream.gyro)

                if not color_frame or not accel_frame or not gyro_frame:
                    continue

                # Record color frame
                color_image = np.asanyarray(color_frame.get_data())
                self.video_writer.write(color_image)

                # Record IMU data
                accel_data = accel_frame.as_motion_frame().get_motion_data()
                gyro_data = gyro_frame.as_motion_frame().get_motion_data()
                timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
                imu_writer.writerow([timestamp, accel_data.x, accel_data.y, accel_data.z, gyro_data.x, gyro_data.y, gyro_data.z])

                # Display the RGB stream (optional)
                cv2.imshow('RealSense', color_image)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        finally:
            # Clean up
            self.pipeline.stop()
            self.video_writer.release()
            self.imu_file.close()
            cv2.destroyAllWindows()

if __name__ == "__main__":
    recorder = RealSenseRecorder()
    recorder.start_recording()
