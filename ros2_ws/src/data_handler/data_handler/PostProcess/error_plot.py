#!/usr/bin/env python3
import argparse
import numpy as np
import matplotlib.pyplot as plt
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import TwistStamped, Vector3Stamped
from scipy.interpolate import interp1d

def main():
    # Parse command-line argument for bag file path
    parser = argparse.ArgumentParser(description='Create error plot from ROS bag file')
    parser.add_argument('bag_file', help='Path to the ROS bag file')
    args = parser.parse_args()

    # Initialize bag file reader
    reader = SequentialReader()
    storage_options = StorageOptions(uri=args.bag_file, storage_id='sqlite3')
    converter_options = ConverterOptions('', '')
    reader.open(storage_options, converter_options)

    # Lists to store timestamps and velocities
    gt_times, gt_vals = [], []  # Ground truth (/motor/present_velocity)
    kf_times, kf_vals = [], []  # Kalman filter (/kalman_filter/state)

    # Extract data from bag file
    while reader.has_next():
        topic, data, t = reader.read_next()
        ts = t / 1e9  # Convert nanoseconds to seconds
        if topic == '/motor/present_velocity':
            msg = deserialize_message(data, TwistStamped)
            gt_times.append(ts)
            gt_vals.append(msg.twist.linear.x)
        elif topic == '/kalman_filter/state':
            msg = deserialize_message(data, Vector3Stamped)
            kf_times.append(ts)
            kf_vals.append(msg.vector.y)

    # Check if data is available
    if not gt_times or not kf_times:
        print("Error: One or both topics have no data")
        return

    # Define common time range
    start_time = max(gt_times[0], kf_times[0])
    end_time = min(gt_times[-1], kf_times[-1])
    if start_time >= end_time:
        print("Error: No overlapping time range between topics")
        return

    # Create time grid
    dt = 0.01  # 10 ms step
    t_grid = np.arange(start_time, end_time, dt)

    # Interpolate both signals to the time grid
    gt_interp = interp1d(gt_times, gt_vals, bounds_error=False, fill_value="extrapolate")(t_grid)
    kf_interp = interp1d(kf_times, kf_vals, bounds_error=False, fill_value="extrapolate")(t_grid)

    # Compute error
    error = gt_interp - kf_interp

    # Shift time to start at zero
    t_shifted = t_grid - t_grid[0]

    # Create the error plot
    plt.figure(figsize=(12, 6))
    plt.plot(t_shifted, error, label='Error (Ground Truth - Estimate)')
    plt.axhline(0, color='gray', linestyle='--', label='Zero Error')
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity Error (m/s)')
    plt.title('Error Plot: /kalman_filter/state vs. /motor/present_velocity')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig('error_plot.png')
    print("Error plot saved to 'error_plot.png'")

if __name__ == '__main__':
    main()

# #!/usr/bin/env python3
# import argparse
# import numpy as np
# import matplotlib.pyplot as plt
# from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
# from rclpy.serialization import deserialize_message
# from geometry_msgs.msg import TwistStamped, Vector3Stamped
# from scipy.interpolate import interp1d

# def main():
#     # Parse command-line argument for bag file path
#     parser = argparse.ArgumentParser(description='Create error plots from ROS bag file')
#     parser.add_argument('bag_file', help='Path to the ROS bag file')
#     args = parser.parse_args()

#     # Initialize bag file reader
#     reader = SequentialReader()
#     storage_options = StorageOptions(uri=args.bag_file, storage_id='sqlite3')
#     converter_options = ConverterOptions('', '')
#     reader.open(storage_options, converter_options)

#     # Lists to store timestamps and velocities
#     gt_times, gt_vals = [], []  # Ground truth (/motor/present_velocity)
#     kf_times, kf_vals = [], []  # Kalman filter (/kalman_filter/state)

#     # Extract data from bag file
#     while reader.has_next():
#         topic, data, t = reader.read_next()
#         ts = t / 1e9  # Convert nanoseconds to seconds
#         if topic == '/motor/present_velocity':
#             msg = deserialize_message(data, TwistStamped)
#             gt_times.append(ts)
#             gt_vals.append(msg.twist.linear.x)
#         elif topic == '/kalman_filter/state':
#             msg = deserialize_message(data, Vector3Stamped)
#             kf_times.append(ts)
#             kf_vals.append(msg.vector.y)

#     # Check if data is available
#     if not gt_times or not kf_times:
#         print("Error: One or both topics have no data")
#         return

#     # Define common time range
#     start_time = max(gt_times[0], kf_times[0])
#     end_time = min(gt_times[-1], kf_times[-1])
#     if start_time >= end_time:
#         print("Error: No overlapping time range between topics")
#         return

#     # Create time grid
#     dt = 0.01  # 10 ms step
#     t_grid = np.arange(start_time, end_time, dt)

#     # Interpolate both signals to the time grid
#     gt_interp = interp1d(gt_times, gt_vals, bounds_error=False, fill_value="extrapolate")(t_grid)
#     kf_interp = interp1d(kf_times, kf_vals, bounds_error=False, fill_value="extrapolate")(t_grid)

#     # Compute error
#     error = gt_interp - kf_interp

#     # Shift time to start at zero
#     t_shifted = t_grid - t_grid[0]

#     # Create figure with two subplots
#     fig, axes = plt.subplots(2, 1, figsize=(12, 10))

#     # Plot 1: Velocity Comparison
#     axes[0].plot(t_shifted, gt_interp, label='Ground Truth', color='blue')
#     axes[0].plot(t_shifted, kf_interp, label='Kalman Filter Estimate', color='orange')
#     axes[0].fill_between(t_shifted, gt_interp, kf_interp, color='gray', alpha=0.2)
#     axes[0].set_xlabel('Time (s)')
#     axes[0].set_ylabel('Velocity (m/s)')
#     axes[0].set_title('Velocity Comparison')
#     axes[0].legend()
#     axes[0].grid(True)

#     # Plot 2: Error Over Time
#     axes[1].plot(t_shifted, error, label='Error (GT - KF)', color='red')
#     axes[1].axhline(0, color='gray', linestyle='--', label='Zero Error')
#     axes[1].set_xlabel('Time (s)')
#     axes[1].set_ylabel('Velocity Error (m/s)')
#     axes[1].set_title('Error Over Time')
#     axes[1].legend()
#     axes[1].grid(True)

#     # Adjust layout and save
#     plt.tight_layout()
#     plt.savefig('error_analysis.png')
#     print("Error analysis plots saved to 'error_analysis.png'")

# if __name__ == '__main__':
#     main()