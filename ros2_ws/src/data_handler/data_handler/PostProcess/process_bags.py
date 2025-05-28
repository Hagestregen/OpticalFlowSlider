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
from std_msgs.msg import Float64  # Added for IMU filtered velocity
from scipy.interpolate import interp1d
from scipy.signal import correlate
import argparse

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
            if topic == "/kalman_filter/imu_filtered":
                msg = deserialize_message(data, Float64)
                meas_data[topic][0].append(ts)
                meas_data[topic][1].append(msg.data)
            elif topic == "/kalman_filter/state":
                msg = deserialize_message(data, Vector3Stamped)
                meas_data[topic][0].append(ts)
                meas_data[topic][1].append(msg.vector.y)  # Velocity is in vector.y
            else:
                msg = deserialize_message(data, Vector3Stamped)
                meas_data[topic][0].append(ts)
                meas_data[topic][1].append(msg.vector.x)

    gt = (np.array(gt_times), np.array(gt_vals))
    meas = {topic: (np.array(times), np.array(vals)) for topic, (times, vals) in meas_data.items()}
    return gt, meas

def interpolate_to_grid(times, values, t_grid):
    if len(times) < 2:
        raise ValueError("Need at least 2 points for interpolation")
    f = interp1d(times, values, bounds_error=False, fill_value="extrapolate")
    return f(t_grid)

def find_optimal_shift(gt_grid, m_grid, dt):
    corr = correlate(gt_grid - gt_grid.mean(), m_grid - m_grid.mean(), mode='full')
    lags = np.arange(-len(m_grid) + 1, len(gt_grid))
    idx = np.argmax(corr)
    tau = lags[idx] * dt
    steps = int(round(tau / dt))
    return steps, tau

def compute_rmse(a, b):
    n = min(len(a), len(b))
    return np.sqrt(np.mean((a[:n] - b[:n]) ** 2))

def compute_mae(a, b):
    n = min(len(a), len(b))
    return np.mean(np.abs(a[:n] - b[:n]))

def plot_aligned_data(t_grid, gt_g, m_g, steps, method, topic_name, resolution, run, output_dir, y_lim=None):
    if steps > 0:
        t_m = t_grid[:-steps]
        m_al = m_g[:-steps]
        g_al = gt_g[steps:]
    else:
        t_m = t_grid[-steps:]
        m_al = m_g[-steps:]
        g_al = gt_g[:steps]

    t_m_shifted = t_m - t_m[0]  # Shift time to start at 0

    # Determine color based on topic_name
    if "IMU" in topic_name:
        color = "orange"  # IMU signal
    elif "KF" in topic_name:
        color = "green"   # Kalman Filter signal
    elif "smooth" in topic_name.lower():
        color = "purple"  # Optical flow smooth signal
    else:
        color = "blue"    # Default to optical flow signal

    plt.figure(figsize=(12, 6))
    plt.plot(t_m_shifted, g_al, label='Motor Velocity', color='black')  # Motor velocity in black
    plt.plot(t_m_shifted, m_al, label=f'{topic_name}', color=color)  # Measurement with specific color
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity')
    plt.title(f'{topic_name} vs Motor Velocity')
    if y_lim is not None:
        plt.ylim(y_lim)
    plt.legend()
    plt.tight_layout()
    filename = os.path.join(output_dir, f'plot_{method}_{topic_name}_{resolution.replace("x", "_")}_run{run}.png')
    print(f"Saving plot to {filename}")
    plt.savefig(filename)
    plt.close()

# --- Main Processing ---

def process_bag(bag_path, method, resolution, meas_topics, output_dir, y_lim=None, dt=0.01):
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

            steps, tau = find_optimal_shift(gt_g, m_g, dt)
            if steps > 0:
                m_al = m_g[:-steps]
                g_al = gt_g[steps:]
            else:
                m_al = m_g[-steps:]
                g_al = gt_g[:steps]

            rmse = compute_rmse(g_al, m_al)
            mae = compute_mae(g_al, m_al)

            # Generate plot for this topic
            if topic == "/kalman_filter/state":
                topic_name = "KF Velocity"
            elif topic == "/kalman_filter/imu_filtered":
                topic_name = "IMU Filtered Velocity"
            else:
                topic_name = topic.split('/')[-1].replace('_velocity', '').replace('output', 'KF')
            plot_aligned_data(t_grid, gt_g, m_g, steps, method, topic_name, resolution, run_name, output_dir, y_lim)

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

def plot_bar(df, resolution, metric, output_dir, y_lim=None):
    sub_df = df[df['resolution'] == resolution].copy()
    sub_df['method_topic'] = sub_df['method'] + ' - ' + sub_df['topic']
    grouped = sub_df.groupby('method_topic', observed=False)[metric].mean().reset_index()
    grouped = grouped.sort_values('method_topic')

    plt.figure(figsize=(12, 6))
    bars = plt.bar(grouped['method_topic'], grouped[metric])
    plt.xlabel('Method - Topic')
    plt.ylabel(metric.upper())
    plt.title(f'Average {metric.upper()} for {resolution}')
    if y_lim is not None:
        plt.ylim(y_lim)
    plt.xticks(rotation=45, ha='right')

    for bar in bars:
        yval = bar.get_height()
        if np.isfinite(yval):
            plt.text(bar.get_x() + bar.get_width()/2, yval, f'{yval:.4f}', ha='center', va='bottom')

    plt.tight_layout()
    filename = os.path.join(output_dir, f'{metric}_{resolution.replace("x", "_")}_per_topic.png')
    print(f"Saving bar plot to {filename}")
    plt.savefig(filename)
    plt.close()

def main():
    parser = argparse.ArgumentParser(description="Process ROS 2 bag files for optical flow methods.")
    parser.add_argument('--base_dir', type=str, required=True, help="Base directory containing the bag files.")
    parser.add_argument('--methods', type=str, nargs='+', help="List of methods to process (e.g., LK RAFT_Large). If not specified, infer from folder structure.")
    parser.add_argument('--runs', type=str, nargs='+', help="Optional list of specific runs to process.")
    parser.add_argument('--output_dir', type=str, default='.', help="Directory to save output files.")
    parser.add_argument('--resolution', type=str, default='640x480', help="Resolution of the data.")
    parser.add_argument('--y_lim', type=float, nargs=2, help="Optional y-axis limits for plots (lower upper)")
    args = parser.parse_args()

    # Method configuration: method name -> (label, list of topics)
    method_config = {
        "LFN3": ("LiteFlowNet3", ["/optical_flow/LFN3_velocity", "/kalman_filter/imu_filtered", "/kalman_filter/state"]),
        "LFN3_Smooth": ("LiteFlowNet3 Smooth", ["/optical_flow/LFN3_smooth_velocity", "/kalman_filter/imu_filtered", "/kalman_filter/state"]),
        "LK": ("Lukas Kanade", ["/optical_flow/LK_velocity", "/kalman_filter/imu_filtered", "/kalman_filter/state"]),
        "PWC_Net": ("PWC-Net", ["/optical_flow/PWC_velocity"]),
        "RAFT_Large": ("RAFT-Large", [
            "/optical_flow/raft_large_velocity",
            "/optical_flow/raft_large_smooth_velocity",
            "/kalman_filter/imu_filtered",
            "/kalman_filter/state"
        ]),
        "RAFT_Small": ("RAFT-Small", ["/optical_flow/raft_small_velocity"]),
    }

    # Subdirectory mapping: method name -> actual subdirectory name
    subdir_map = {
        "LK": "LucasKanade",
        "RAFT_Large": "RAFT",
        # Add other mappings as needed, e.g., "LFN3": "LiteFlowNet3"
    }

    # If methods are not specified, infer from subdirectories
    if not args.methods:
        method_dirs = [d for d in os.listdir(args.base_dir) if os.path.isdir(os.path.join(args.base_dir, d))]
        args.methods = [m for m in method_dirs if m in method_config or m in subdir_map.values()]
        if not args.methods:
            print(f"No valid methods found in {args.base_dir}. Please specify methods.")
            return

    all_results = []

    for method in args.methods:
        # Get the actual subdirectory name, default to method name if not in subdir_map
        method_subdir = subdir_map.get(method, method)
        method_dir = os.path.join(args.base_dir, method_subdir)
        if not os.path.exists(method_dir):
            print(f"Directory {method_dir} does not exist.")
            continue

        # Find all .db3 files in the method's subdirectory
        bag_paths = glob.glob(os.path.join(method_dir, "**", "*.db3"), recursive=True)
        if not bag_paths:
            print(f"No .db3 files found under {method_dir}")
            continue

        label, meas_topics = method_config.get(method, (method, [f"/optical_flow/{method}_velocity"]))
        for bag_path in bag_paths:
            run_name = os.path.basename(os.path.dirname(bag_path))
            if args.runs and run_name not in args.runs:
                continue

            results = process_bag(bag_path, label, args.resolution, meas_topics, args.output_dir, y_lim=args.y_lim)
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
        plot_bar(df, resolution, 'rmse', args.output_dir, y_lim=args.y_lim)
        plot_bar(df, resolution, 'mae', args.output_dir, y_lim=args.y_lim)

if __name__ == "__main__":
    main()