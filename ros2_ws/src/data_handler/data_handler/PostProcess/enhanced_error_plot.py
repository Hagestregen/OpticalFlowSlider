#!/usr/bin/env python3
import argparse
import numpy as np
import matplotlib.pyplot as plt
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import TwistStamped, Vector3Stamped
from scipy.interpolate import interp1d
from scipy.signal import correlate, welch

def main():
    # Parse command-line argument for bag file path
    parser = argparse.ArgumentParser(description='Create error plots from ROS bag file')
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

    # Create a figure with subplots (4 rows, 2 columns)
    fig, axes = plt.subplots(4, 2, figsize=(16, 16))
    fig.suptitle('Comprehensive Error Analysis: Kalman Filter vs. Motor Velocity', fontsize=16)

    # Plot 1: Time Series Plot of Error
    axes[0, 0].plot(t_shifted, error, label='Error (GT - KF)')
    axes[0, 0].axhline(0, color='gray', linestyle='--', label='Zero Error')
    axes[0, 0].set_xlabel('Time (s)')
    axes[0, 0].set_ylabel('Velocity Error (m/s)')
    axes[0, 0].set_title('Error Over Time')
    axes[0, 0].legend()
    axes[0, 0].grid(True)

    # Plot 2: Histogram of Error
    axes[0, 1].hist(error, bins=50, color='skyblue', edgecolor='black')
    axes[0, 1].set_xlabel('Velocity Error (m/s)')
    axes[0, 1].set_ylabel('Frequency')
    axes[0, 1].set_title('Error Distribution')
    axes[0, 1].grid(True)

    # Plot 3: Scatter Plot of Estimated vs. Actual Velocity
    axes[1, 0].scatter(gt_interp, kf_interp, alpha=0.5)
    axes[1, 0].plot([min(gt_interp), max(gt_interp)], [min(gt_interp), max(gt_interp)], 'r--', label='Perfect Correlation')
    axes[1, 0].set_xlabel('Ground Truth Velocity (m/s)')
    axes[1, 0].set_ylabel('Kalman Filter Velocity (m/s)')
    axes[1, 0].set_title('Estimated vs. Actual Velocity')
    axes[1, 0].legend()
    axes[1, 0].grid(True)

    # Plot 4: Residual Plot
    axes[1, 1].scatter(t_shifted, error, alpha=0.5)
    axes[1, 1].axhline(0, color='gray', linestyle='--')
    axes[1, 1].set_xlabel('Time (s)')
    axes[1, 1].set_ylabel('Velocity Error (m/s)')
    axes[1, 1].set_title('Residual Plot')
    axes[1, 1].grid(True)

    # Plot 5: Cross-Correlation Plot
    cross_corr = correlate(gt_interp - np.mean(gt_interp), kf_interp - np.mean(kf_interp), mode='full')
    lags = np.arange(-len(t_shifted) + 1, len(t_shifted))
    time_lags = lags * dt
    axes[2, 0].plot(time_lags, cross_corr)
    axes[2, 0].set_xlabel('Time Lag (s)')
    axes[2, 0].set_ylabel('Cross-Correlation')
    axes[2, 0].set_title('Cross-Correlation of Ground Truth and KF')
    axes[2, 0].grid(True)

    # Plot 6: Cumulative Error Plot
    cumulative_error = np.cumsum(error) * dt
    axes[2, 1].plot(t_shifted, cumulative_error, color='purple')
    axes[2, 1].axhline(0, color='gray', linestyle='--')
    axes[2, 1].set_xlabel('Time (s)')
    axes[2, 1].set_ylabel('Cumulative Error (m/s·s)')
    axes[2, 1].set_title('Cumulative Error Over Time')
    axes[2, 1].grid(True)

    # Plot 7: Power Spectral Density (PSD)
    fs = 1 / dt  # Sampling frequency
    f_gt, psd_gt = welch(gt_interp, fs=fs, nperseg=256)
    f_kf, psd_kf = welch(kf_interp, fs=fs, nperseg=256)
    axes[3, 0].semilogy(f_gt, psd_gt, label='Ground Truth')
    axes[3, 0].semilogy(f_kf, psd_kf, label='Kalman Filter')
    axes[3, 0].set_xlabel('Frequency (Hz)')
    axes[3, 0].set_ylabel('Power Spectral Density')
    axes[3, 0].set_title('Power Spectral Density')
    axes[3, 0].legend()
    axes[3, 0].grid(True)

    # Plot 8: Box Plot of Error
    axes[3, 1].boxplot(error, vert=True, patch_artist=True)
    axes[3, 1].set_ylabel('Velocity Error (m/s)')
    axes[3, 1].set_title('Box Plot of Error')
    axes[3, 1].grid(True)

    # Adjust layout and save the figure
    plt.tight_layout(rect=[0, 0, 1, 0.95])
    plt.savefig('enhanced_error_plots.png')
    print("Enhanced error plots saved to 'enhanced_error_plots.png'")

if __name__ == '__main__':
    main()