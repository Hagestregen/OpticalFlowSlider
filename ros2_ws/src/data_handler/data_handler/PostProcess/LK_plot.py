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
import argparse

# Define the desired order of methods for plotting
method_order = ["Lukas Kanade", "PWC-Net", "LiteFlowNet3", "RAFT-Small", "RAFT-Large"]

# --- Helper Functions ---

def extract_velocity_data(bag_path, gt_topic, meas_topics):
    reader = SequentialReader()
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions('', '')
    reader.open(storage_options, converter_options)

    gt_times, gt_vals = [], []
    meas_data = {topic: ([], []) for topic in meas_topics}  # (times, values) per topic

    while reader.has_next():
        topic, data, t = reader.read_next()
        ts = t / 1e9
        if topic == gt_topic:
            msg = deserialize_message(data, TwistStamped)
            gt_times.append(ts)
            gt_vals.append(msg.twist.linear.x)
        elif topic in meas_topics:
            msg = deserialize_message(data, Vector3Stamped)
            meas_data[topic][0].append(ts)  # times
            meas_data[topic][1].append(msg.vector.x)  # values

    gt = (np.array(gt_times), np.array(gt_vals))
    meas = {topic: (np.array(times), np.array(vals)) for topic, (times, vals) in meas_data.items()}
    return gt, meas

def interpolate_to_grid(times, values, t_grid):
    if len(times) < 2:
        raise ValueError("Need at least 2 points for interpolation")
    f = interp1d(times, values, bounds_error=False, fill_value="extrapolate")
    return f(t_grid)

def compute_rmse(a, b):
    n = min(len(a), len(b))
    return np.sqrt(np.mean((a[:n] - b[:n]) ** 2))

def compute_mae(a, b):
    n = min(len(a), len(b))
    return np.mean(np.abs(a[:n] - b[:n]))

def plot_data(t_grid, gt_g, m_g, method, topic_name, resolution, run, output_dir):
    t_shifted = t_grid - t_grid[0]  # Shift time to start at 0

    plt.figure(figsize=(12, 6))
    plt.plot(t_shifted, gt_g, label='Motor Velocity', color='black')
    plt.plot(t_shifted, m_g, label=f'{method} ({topic_name})', color='blue')
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity')
    plt.title(f'{topic_name} vs Motor Velocity ({resolution})')
    plt.legend()
    plt.tight_layout()
    filename = os.path.join(output_dir, f'plot_{method}_{topic_name}_{resolution.replace("x", "_")}_run{run}.png')
    print(f"Saving plot to {filename}")
    plt.savefig(filename)
    plt.close()

# --- Main Processing ---

def process_bag(bag_path, method, resolution, meas_topics, output_dir, dt=0.01):
    try:
        gt, meas = extract_velocity_data(bag_path, "/motor/present_velocity", meas_topics)
        if len(gt[0]) < 2:
            raise ValueError("Insufficient ground truth data points")

        results = []
        run_name = os.path.basename(os.path.dirname(bag_path))

        for topic in meas_topics:
            m_t, m_v = meas.get(topic, ([], []))
            if len(m_t) < 2:
                print(f"Skipping {topic} in {bag_path}: Insufficient data points")
                continue

            t0 = max(gt[0][0], m_t[0])
            t1 = min(gt[0][-1], m_t[-1])
            if t1 <= t0:
                print(f"Skipping {topic} in {bag_path}: No time overlap")
                continue

            t_grid = np.arange(t0, t1, dt)
            gt_g = interpolate_to_grid(gt[0], gt[1], t_grid)
            m_g = interpolate_to_grid(m_t, m_v, t_grid)

            # Compute metrics directly without alignment
            rmse = compute_rmse(gt_g, m_g)
            mae = compute_mae(gt_g, m_g)

            # Generate plot for this topic
            topic_name = topic.split('/')[-1].replace('_velocity', '').replace('output', 'KF')
            plot_data(t_grid, gt_g, m_g, method, topic_name, resolution, run_name, output_dir)

            results.append({
                'method': method,
                'topic': topic_name,
                'resolution': resolution,
                'run': run_name,
                'rmse': rmse,
                'mae': mae
            })

        return results
    except Exception as e:
        print(f"Error processing {bag_path}: {e}")
        return []

def plot_bar(df, resolution, metric, method_order, output_dir):
    sub_df = df[df['resolution'] == resolution].copy()
    sub_df['method'] = pd.Categorical(sub_df['method'], categories=method_order, ordered=True)
    sub_df = sub_df.sort_values('method')
    
    # Add observed=False to silence the warning
    grouped = sub_df.groupby('method', observed=False)[metric].mean().reset_index()

    plt.figure(figsize=(12, 6))
    bars = plt.bar(grouped['method'], grouped[metric])
    plt.xlabel('Method')
    plt.ylabel(metric.upper())
    plt.title(f'Average {metric.upper()} for {resolution} with Stationary Lighting')
    plt.xticks(rotation=45, ha='right')

    # Only add text for finite values
    for bar in bars:
        yval = bar.get_height()
        if np.isfinite(yval):  # Skip NaN or inf
            plt.text(bar.get_x() + bar.get_width()/2, yval, f'{yval:.4f}', ha='center', va='bottom')

    plt.tight_layout()
    filename = os.path.join(output_dir, f'{metric}_{resolution.replace("x", "_")}.png')
    print(f"Saving bar plot to {filename}")
    plt.savefig(filename)
    plt.close()

def main():
    parser = argparse.ArgumentParser(description="Process ROS 2 bag files for optical flow methods without alignment.")
    parser.add_argument('--base_dir', type=str, required=True, help="Base directory containing the bag files.")
    parser.add_argument('--methods', type=str, nargs='+', help="List of methods to process (e.g., LFN3 LK). If not specified, infer from folder structure.")
    parser.add_argument('--runs', type=str, nargs='+', help="Optional list of specific runs to process.")
    parser.add_argument('--output_dir', type=str, default='.', help="Directory to save output files.")
    parser.add_argument('--resolution', type=str, default='640x480', help="Resolution of the data.")
    args = parser.parse_args()

    # Method configuration: method name -> (label, list of topics)
    method_config = {
        "LFN3": ("LiteFlowNet3", ["/optical_flow/LFN3_velocity", "/imu/integrated_velocity", "/kalman_filter/output"]),
        "LFN3": ("LiteFlowNet3 Smooth", ["/optical_flow/LFN3_smooth_velocity", "/imu/integrated_velocity", "/kalman_filter/output"]),
        "KF": ("Lukas Kanade", ["/optical_flow/LK_velocity", "/imu/integrated_velocity", "/kalman_filter/output"]),
        "PWC_Net": ("PWC-Net", ["/optical_flow/PWC_velocity"]),
        "RAFT_Large": ("RAFT-Large", ["/optical_flow/raft_large_velocity"]),
        "RAFT_Small": ("RAFT-Small", ["/optical_flow/raft_small_velocity"]),
    }

    # If methods are not specified, infer from subdirectories
    if not args.methods:
        method_dirs = [d for d in os.listdir(args.base_dir) if os.path.isdir(os.path.join(args.base_dir, d))]
        args.methods = [d for d in method_dirs if d in method_config]
        if not args.methods:
            print(f"No valid methods found in {args.base_dir}. Please specify methods.")
            return

    all_results = []

    for method in args.methods:
        method_dir = os.path.join(args.base_dir, method) if method in os.listdir(args.base_dir) else args.base_dir
        if not os.path.exists(method_dir):
            print(f"Directory {method_dir} does not exist.")
            continue

        # Find all .db3 files
        bag_paths = glob.glob(os.path.join(method_dir, "**", "*.db3"), recursive=True)
        if not bag_paths:
            print(f"No .db3 files found under {method_dir}")
            continue

        label, meas_topics = method_config.get(method, (method, [f"/optical_flow/{method}_velocity"]))
        for bag_path in bag_paths:
            run_name = os.path.basename(os.path.dirname(bag_path))
            if args.runs and run_name not in args.runs:
                continue

            results = process_bag(bag_path, label, args.resolution, meas_topics, args.output_dir)
            all_results.extend(results)

    if not all_results:
        print("No results collected. Exiting.")
        return

    # Convert results to DataFrame
    df = pd.DataFrame(all_results)

    # Save and display per-run metrics
    df.to_csv(os.path.join(args.output_dir, "metrics_per_run.csv"), index=False)
    print("### Per-run Metrics")
    print(df.to_string(index=False))

    # Compute and save average metrics
    avg_df = df.groupby(['method', 'topic', 'resolution']).mean(numeric_only=True).reset_index()
    avg_df = avg_df[['method', 'topic', 'resolution', 'rmse', 'mae']]
    avg_df.to_csv(os.path.join(args.output_dir, "metrics_average.csv"), index=False)
    print("\n### Average Metrics")
    print(avg_df.to_string(index=False))

    # Generate bar plots for available resolutions
    for resolution in df['resolution'].unique():
        plot_bar(df, resolution, 'rmse', method_order, args.output_dir)
        plot_bar(df, resolution, 'mae', method_order, args.output_dir)

if __name__ == "__main__":
    main()