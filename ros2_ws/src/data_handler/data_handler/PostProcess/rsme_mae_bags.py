#!/usr/bin/env python3
import os
import glob
import csv
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend
import matplotlib.pyplot as plt
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from geometry_msgs.msg import TwistStamped, Vector3Stamped
from scipy.interpolate import interp1d
from scipy.signal import correlate
import argparse

# Define the desired order of methods for bar plots
method_order = ["Lukas Kanade", "PWC-Net", "LiteFlowNet3", "RAFT-Small", "RAFT-Large"]

# --- Helper Functions ---

def extract_velocity_data(bag_path, gt_topic, meas_topic):
    """Extract velocity data from a ROS 2 bag file."""
    reader = SequentialReader()
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions('', '')
    reader.open(storage_options, converter_options)

    gt_times, gt_vals = [], []
    meas_times, meas_vals = [], []

    while reader.has_next():
        topic, data, t = reader.read_next()
        ts = t / 1e9  # Convert nanoseconds to seconds
        if topic == gt_topic:
            msg = deserialize_message(data, TwistStamped)
            gt_times.append(ts)
            gt_vals.append(msg.twist.linear.x)
        elif topic == meas_topic:
            msg = deserialize_message(data, Vector3Stamped)
            meas_times.append(ts)
            meas_vals.append(msg.vector.x)

    return (np.array(gt_times), np.array(gt_vals)), (np.array(meas_times), np.array(meas_vals))

def interpolate_to_grid(times, values, t_grid):
    """Interpolate data onto a uniform time grid."""
    if len(times) < 2:
        raise ValueError("Need at least 2 points for interpolation")
    f = interp1d(times, values, bounds_error=False, fill_value="extrapolate")
    return f(t_grid)

def find_optimal_shift(gt_grid, m_grid, dt):
    """Find the optimal time shift between two signals using cross-correlation."""
    corr = correlate(gt_grid - gt_grid.mean(), m_grid - m_grid.mean(), mode='full')
    lags = np.arange(-len(m_grid) + 1, len(gt_grid))
    idx = np.argmax(corr)
    tau = lags[idx] * dt
    steps = int(round(tau / dt))
    return steps, tau

def compute_rmse(a, b):
    """Compute the Root Mean Square Error between two arrays."""
    n = min(len(a), len(b))
    return np.sqrt(np.mean((a[:n] - b[:n]) ** 2))

def compute_mae(a, b):
    """Compute the Mean Absolute Error between two arrays."""
    n = min(len(a), len(b))
    return np.mean(np.abs(a[:n] - b[:n]))

def plot_aligned_data(t_grid, gt_g, m_g, steps, label, resolution, run, ylim=None):
    """Plot aligned ground truth and measured data with optional y-axis limits."""
    if steps > 0:
        t_m = t_grid[:-steps]
        m_al = m_g[:-steps]
        g_al = gt_g[steps:]
    else:
        t_m = t_grid[-steps:]
        m_al = m_g[-steps:]
        g_al = gt_g[:steps]

    t_m_shifted = t_m - t_m[0]  # Shift time to start at 0

    plt.figure(figsize=(12, 6))
    plt.plot(t_m_shifted, g_al, label='Motor Velocity', color='black')
    plt.plot(t_m_shifted, m_al, label=f'{label} Flow (Shifted)', color='blue')
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity')
    plt.title(f'{label} Flow vs Motor Velocity ({resolution}, Run {run})')
    if ylim is not None:
        plt.ylim(ylim)  # Set y-axis limits if provided
    plt.legend()
    plt.tight_layout()
    filename = f'plot_{label}_{resolution.replace("x", "_")}_run{run}.png'
    print(f"Saving plot to {filename}")
    plt.savefig(filename)
    plt.close()

def process_bag(bag_path, label, meas_topic, resolution, dt=0.01, ylim=None):
    """Process a single bag file and compute alignment metrics."""
    bag_dir = os.path.basename(os.path.dirname(bag_path))
    parts = bag_dir.split('_')
    run_number = parts[-2] if len(parts) > 1 else 'unknown'

    try:
        (gt_t, gt_v), (m_t, m_v) = extract_velocity_data(bag_path, "/motor/present_velocity", meas_topic)
        if len(gt_t) < 2 or len(m_t) < 2:
            raise ValueError("Insufficient data points")

        t0 = max(gt_t[0], m_t[0])
        t1 = min(gt_t[-1], m_t[-1])
        if t1 <= t0:
            raise ValueError("No time overlap")

        t_grid = np.arange(t0, t1, dt)
        gt_g = interpolate_to_grid(gt_t, gt_v, t_grid)
        m_g = interpolate_to_grid(m_t, m_v, t_grid)

        steps, tau = find_optimal_shift(gt_g, m_g, dt)
        if steps > 0:
            m_al = m_g[:-steps]
            g_al = gt_g[steps:]
        else:
            m_al = m_g[-steps:]
            g_al = gt_g[:steps]

        rmse = compute_rmse(g_al, m_al)
        mae = compute_mae(g_al, m_al)

        # Generate plot for this run
        plot_aligned_data(t_grid, gt_g, m_g, steps, label, resolution, run_number, ylim=ylim)

        return {'method': label, 'resolution': resolution, 'run': run_number, 'rmse': rmse, 'mae': mae}
    except Exception as e:
        print(f"Error processing {bag_path}: {e}")
        return None

def plot_bar(df, resolution, metric, method_order):
    """Generate a bar plot for a given metric and resolution."""
    sub_df = df[df['resolution'] == resolution].copy()
    sub_df['method'] = pd.Categorical(sub_df['method'], categories=method_order, ordered=True)
    sub_df = sub_df.sort_values('method')
    methods = sub_df['method']
    values = sub_df[metric]

    plt.figure(figsize=(12, 6))
    bars = plt.bar(methods, values)
    plt.xlabel('Method')
    plt.ylabel(metric.upper())
    plt.title(f'{metric.upper()} for {resolution} with Moving Lighting')
    plt.xticks(rotation=45, ha='right')

    for bar in bars:
        yval = bar.get_height()
        plt.text(bar.get_x() + bar.get_width()/2, yval, f'{yval:.4f}', ha='center', va='bottom')

    plt.tight_layout()
    filename = f'{metric}_{resolution.replace("x", "_")}.png'
    print(f"Saving bar plot to {filename}")
    plt.savefig(filename)
    plt.close()

def main():
    """Main function to process bag files and generate metrics and plots."""
    parser = argparse.ArgumentParser(description="Process ROS 2 bag files and analyze optical flow methods.")
    parser.add_argument('--base_dir', type=str, required=True, help="Base directory containing ROS 2 bag files")
    parser.add_argument('--resolution', type=str, required=True, help="Resolution of the data, e.g., 640x480")
    parser.add_argument('--ylim', type=float, nargs=2, default=None, 
                        help="Set y-axis limits for velocity plots, e.g., --ylim -1.0 1.0 (min max)")
    args = parser.parse_args()

    # Method configuration: directory name -> (label, topic)
    method_config = {
        "LFN3": ("LiteFlowNet3", "/optical_flow/LFN3_velocity"),
        "PWC_Net": ("PWC-Net", "/optical_flow/PWC_velocity"),
        "RAFT_Large_Bad_Light_640": ("RAFT-Large", "/optical_flow/raft_large_smooth_velocity"),
        "RAFT_Small": ("RAFT-Small", "/optical_flow/raft_small_velocity"),
        "Lucas_Kanade_Light": ("Lukas Kanade", "/optical_flow/LK_velocity"),
    }

    # Extract method_dir from base_dir
    method_dir = os.path.basename(args.base_dir)
    if method_dir not in method_config:
        print(f"Unknown method directory: {method_dir}")
        return

    label, meas_topic = method_config[method_dir]

    results = []

    # Process bags
    bag_paths = glob.glob(os.path.join(args.base_dir, "**", "*.db3"), recursive=True)
    if not bag_paths:
        print(f"No .db3 files found under {args.base_dir}")
        return

    for bag_path in bag_paths:
        result = process_bag(bag_path, label, meas_topic, args.resolution, ylim=args.ylim)
        if result:
            results.append(result)

    # Check if results are empty
    if not results:
        print("No results collected. Exiting.")
        return

    # Convert results to DataFrame
    df = pd.DataFrame(results)

    # Save and display per-run metrics
    df.to_csv("metrics_per_run.csv", index=False)
    print("### Per-run Metrics")
    print(df.to_string(index=False))

    # Compute and save average metrics
    avg_df = df.groupby(['method', 'resolution']).mean(numeric_only=True).reset_index()
    avg_df = avg_df[['method', 'resolution', 'rmse', 'mae']]

    # Sort avg_df by the custom method order
    avg_df['method'] = pd.Categorical(avg_df['method'], categories=method_order, ordered=True)
    avg_df = avg_df.sort_values('method')

    avg_df.to_csv("metrics_average.csv", index=False)
    print("\n### Average Metrics")
    print(avg_df.to_string(index=False))

    # Generate bar plots for available resolutions with custom order
    for resolution in df['resolution'].unique():
        plot_bar(avg_df, resolution, 'rmse', method_order)
        plot_bar(avg_df, resolution, 'mae', method_order)

if __name__ == "__main__":
    main()
# import os
# import glob
# import csv
# import numpy as np
# import pandas as pd
# import matplotlib
# matplotlib.use('Agg')  # Use non-interactive backend
# import matplotlib.pyplot as plt
# from rclpy.serialization import deserialize_message
# from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
# from geometry_msgs.msg import TwistStamped, Vector3Stamped
# from scipy.interpolate import interp1d
# from scipy.signal import correlate

# # Define the desired order of methods
# method_order = ["Lukas Kanade", "PWC-Net", "LiteFlowNet3", "RAFT-Small", "RAFT-Large"]

# # --- Helper Functions ---

# def extract_velocity_data(bag_path, gt_topic, meas_topic):
#     reader = SequentialReader()
#     storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
#     converter_options = ConverterOptions('', '')
#     reader.open(storage_options, converter_options)

#     gt_times, gt_vals = [], []
#     meas_times, meas_vals = [], []

#     while reader.has_next():
#         topic, data, t = reader.read_next()
#         ts = t / 1e9
#         if topic == gt_topic:
#             msg = deserialize_message(data, TwistStamped)
#             gt_times.append(ts)
#             gt_vals.append(msg.twist.linear.x)
#         elif topic == meas_topic:
#             msg = deserialize_message(data, Vector3Stamped)
#             meas_times.append(ts)
#             meas_vals.append(msg.vector.x)

#     return (np.array(gt_times), np.array(gt_vals)), (np.array(meas_times), np.array(meas_vals))

# def interpolate_to_grid(times, values, t_grid):
#     if len(times) < 2:
#         raise ValueError("Need at least 2 points for interpolation")
#     f = interp1d(times, values, bounds_error=False, fill_value="extrapolate")
#     return f(t_grid)

# def find_optimal_shift(gt_grid, m_grid, dt):
#     corr = correlate(gt_grid - gt_grid.mean(), m_grid - m_grid.mean(), mode='full')
#     lags = np.arange(-len(m_grid) + 1, len(gt_grid))
#     idx = np.argmax(corr)
#     tau = lags[idx] * dt
#     steps = int(round(tau / dt))
#     return steps, tau

# def compute_rmse(a, b):
#     n = min(len(a), len(b))
#     return np.sqrt(np.mean((a[:n] - b[:n]) ** 2))

# def compute_mae(a, b):
#     n = min(len(a), len(b))
#     return np.mean(np.abs(a[:n] - b[:n]))

# def plot_aligned_data(t_grid, gt_g, m_g, steps, method, resolution, run):
#     if steps > 0:
#         t_m = t_grid[:-steps]
#         m_al = m_g[:-steps]
#         g_al = gt_g[steps:]
#     else:
#         t_m = t_grid[-steps:]
#         m_al = m_g[-steps:]
#         g_al = gt_g[:steps]

#     t_m_shifted = t_m - t_m[0]  # Shift time to start at 0

#     plt.figure(figsize=(12, 6))
#     plt.plot(t_m_shifted, g_al, label='Motor Velocity', color='black')
#     plt.plot(t_m_shifted, m_al, label=f'{method} Flow (Shifted)', color='blue')
#     plt.xlabel('Time (s)')
#     plt.ylabel('Velocity')
#     plt.title(f'{method} Flow vs Motor Velocity ({resolution}, Run {run})')
#     plt.legend()
#     plt.tight_layout()
#     filename = f'plot_{method}_{resolution.replace("x", "_")}_run{run}.png'
#     print(f"Saving plot to {filename}")
#     plt.savefig(filename)
#     plt.close()

# # --- Main Processing ---

# def process_bag(bag_path, method_dir, resolution, method_config, dt=0.01):
#     label, meas_topic = method_config[method_dir]
#     bag_dir = os.path.basename(os.path.dirname(bag_path))
#     parts = bag_dir.split('_')
#     run_number = parts[-2]

#     try:
#         (gt_t, gt_v), (m_t, m_v) = extract_velocity_data(bag_path, "/motor/present_velocity", meas_topic)
#         if len(gt_t) < 2 or len(m_t) < 2:
#             raise ValueError("Insufficient data points")

#         t0 = max(gt_t[0], m_t[0])
#         t1 = min(gt_t[-1], m_t[-1])
#         if t1 <= t0:
#             raise ValueError("No time overlap")

#         t_grid = np.arange(t0, t1, dt)
#         gt_g = interpolate_to_grid(gt_t, gt_v, t_grid)
#         m_g = interpolate_to_grid(m_t, m_v, t_grid)

#         steps, tau = find_optimal_shift(gt_g, m_g, dt)
#         if steps > 0:
#             m_al = m_g[:-steps]
#             g_al = gt_g[steps:]
#         else:
#             m_al = m_g[-steps:]
#             g_al = gt_g[:steps]

#         rmse = compute_rmse(g_al, m_al)
#         mae = compute_mae(g_al, m_al)

#         # Generate plot for this run
#         plot_aligned_data(t_grid, gt_g, m_g, steps, label, resolution, run_number)

#         return {'method': label, 'resolution': resolution, 'run': run_number, 'rmse': rmse, 'mae': mae}
#     except Exception as e:
#         print(f"Error processing {bag_path}: {e}")
#         return None

# def plot_bar(df, resolution, metric, method_order):
#     sub_df = df[df['resolution'] == resolution].copy()
#     sub_df['method'] = pd.Categorical(sub_df['method'], categories=method_order, ordered=True)
#     sub_df = sub_df.sort_values('method')
#     methods = sub_df['method']
#     values = sub_df[metric]

#     plt.figure(figsize=(12, 6))
#     bars = plt.bar(methods, values)
#     plt.xlabel('Method')
#     plt.ylabel(metric.upper())
#     plt.title(f'{metric.upper()} for {resolution} with Moving Lighting')
#     plt.xticks(rotation=45, ha='right')

#     for bar in bars:
#         yval = bar.get_height()
#         plt.text(bar.get_x() + bar.get_width()/2, yval, f'{yval:.4f}', ha='center', va='bottom')

#     plt.tight_layout()
#     filename = f'{metric}_{resolution.replace("x", "_")}.png'
#     print(f"Saving bar plot to {filename}")
#     plt.savefig(filename)
#     plt.close()

# def main():
#     # Method configuration: directory name -> (label, topic)
#     method_config = {
#         "LFN3": ("LiteFlowNet3", "/optical_flow/LFN3_velocity"),
#         "PWC_Net": ("PWC-Net", "/optical_flow/PWC_velocity"),
#         "RAFT_Large": ("RAFT-Large", "/optical_flow/raft_large_velocity"),
#         "RAFT_Small": ("RAFT-Small", "/optical_flow/raft_small_velocity"),
#         # "Lucas_Kanade_Heavy": ("LK_V3", "/optical_flow/LK_velocity"),
#         "Lucas_Kanade_Light": ("Lukas Kanade", "/optical_flow/LK_velocity"),
#         # "Lucas_Kanade_Medium": ("LK_V2", "/optical_flow/LK_velocity"),
#     }

#     # Base directory (only one path for now)
#     base_dir_640 = "/home/docker/OpticalFlowSlider/ros2_ws/my_rosbag/Experiment3Occlusion/Result_640/LFN3"

#     results = []

#     # Process bags for the single resolution
#     for base_dir, resolution in [(base_dir_640, "640x480")]:
#         bag_paths = glob.glob(os.path.join(base_dir, "**", "*.db3"), recursive=True)
#         if not bag_paths:
#             print(f"No .db3 files found under {base_dir}")
#             continue

#         for bag_path in bag_paths:
#             method_dir = os.path.basename(os.path.dirname(os.path.dirname(bag_path)))
#             if method_dir not in method_config:
#                 print(f"Unknown method directory: {method_dir}")
#                 continue
#             result = process_bag(bag_path, method_dir, resolution, method_config)
#             if result:
#                 results.append(result)

#     # Check if results are empty
#     if not results:
#         print("No results collected. Exiting.")
#         return

#     # Convert results to DataFrame
#     df = pd.DataFrame(results)

#     # Save and display per-run metrics
#     df.to_csv("metrics_per_run.csv", index=False)
#     print("### Per-run Metrics")
#     print(df.to_string(index=False))

#     # Compute and save average metrics
#     avg_df = df.groupby(['method', 'resolution']).mean(numeric_only=True).reset_index()
#     avg_df = avg_df[['method', 'resolution', 'rmse', 'mae']]

#     # Sort avg_df by the custom method order
#     avg_df['method'] = pd.Categorical(avg_df['method'], categories=method_order, ordered=True)
#     avg_df = avg_df.sort_values('method')

#     avg_df.to_csv("metrics_average.csv", index=False)
#     print("\n### Average Metrics")
#     print(avg_df.to_string(index=False))

#     # Generate bar plots for available resolutions with custom order
#     for resolution in df['resolution'].unique():
#         plot_bar(avg_df, resolution, 'rmse', method_order)
#         plot_bar(avg_df, resolution, 'mae', method_order)

# if __name__ == "__main__":
#     main()
