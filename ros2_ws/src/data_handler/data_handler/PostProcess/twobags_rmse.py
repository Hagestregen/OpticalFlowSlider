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

# --- Helper Functions ---

def extract_velocity_data(bag_path, gt_topic, meas_topic):
    reader = SequentialReader()
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions('', '')
    reader.open(storage_options, converter_options)

    gt_times, gt_vals = [], []
    meas_times, meas_vals = [], []

    while reader.has_next():
        topic, data, t = reader.read_next()
        ts = t / 1e9
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

def plot_aligned_data(t_grid, gt_g, m_g, steps, method, resolution, run):
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
    plt.plot(t_m_shifted, m_al, label=f'{method} Flow (Shifted)', color='blue')
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity')
    plt.title(f'{method} Flow vs Motor Velocity ({resolution}, Run {run})')
    plt.legend()
    plt.tight_layout()
    filename = f'plot_{method}_{resolution.replace("x", "_")}_run{run}.png'
    print(f"Saving plot to {filename}")
    plt.savefig(filename)
    plt.close()

def plot_combined_aligned_data(t_grid_640, gt_g_640, m_g_640, steps_640, t_grid_960, gt_g_960, m_g_960, steps_960, method, run):
    # Handle 640x480 data
    if steps_640 > 0:
        t_m_640 = t_grid_640[:-steps_640]
        m_al_640 = m_g_640[:-steps_640]
        g_al_640 = gt_g_640[steps_640:]
    else:
        t_m_640 = t_grid_640[-steps_640:]
        m_al_640 = m_g_640[-steps_640:]
        g_al_640 = gt_g_640[:steps_640]

    t_m_shifted_640 = t_m_640 - t_m_640[0]  # Shift time to start at 0

    # Handle 960x540 data
    if steps_960 > 0:
        t_m_960 = t_grid_960[:-steps_960]
        m_al_960 = m_g_960[:-steps_960]
        g_al_960 = gt_g_960[steps_960:]
    else:
        t_m_960 = t_grid_960[-steps_960:]
        m_al_960 = m_g_960[-steps_960:]
        g_al_960 = gt_g_960[:steps_960]

    t_m_shifted_960 = t_m_960 - t_m_960[0]  # Shift time to start at 0

    # Find the common time range
    t_start = max(t_m_shifted_640[0], t_m_shifted_960[0])
    t_end = min(t_m_shifted_640[-1], t_m_shifted_960[-1])

    # Create a common time grid
    t_common = np.linspace(t_start, t_end, num=1000)

    # Interpolate both datasets to the common time grid
    interp_gt_640 = interp1d(t_m_shifted_640, g_al_640, bounds_error=False, fill_value="extrapolate")
    interp_m_640 = interp1d(t_m_shifted_640, m_al_640, bounds_error=False, fill_value="extrapolate")
    interp_m_960 = interp1d(t_m_shifted_960, m_al_960, bounds_error=False, fill_value="extrapolate")

    gt_common_640 = interp_gt_640(t_common)
    m_common_640 = interp_m_640(t_common)
    m_common_960 = interp_m_960(t_common)

    # Plot the data with one motor velocity and specified colors
    plt.figure(figsize=(12, 6))
    plt.plot(t_common, gt_common_640, label='Motor Velocity', color='black')
    plt.plot(t_common, m_common_640, label=f'{method} Flow (640x480)', color='blue')
    plt.plot(t_common, m_common_960, label=f'{method} Flow (960x540)', color='orange')
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity')
    plt.title(f'Combined {method} Flow vs Motor Velocity for Run {run}')
    plt.legend()
    plt.tight_layout()
    filename = f'combined_plot_{method}_run{run}.png'
    print(f"Saving combined plot to {filename}")
    plt.savefig(filename)
    plt.close()

def process_bag(bag_path, method_dir, resolution, method_config, dt=0.01):
    label, meas_topic = method_config[method_dir]
    bag_dir = os.path.basename(os.path.dirname(bag_path))
    parts = bag_dir.split('_')
    run_number = parts[-2]

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

        # Comment out individual plot generation
        # plot_aligned_data(t_grid, gt_g, m_g, steps, label, resolution, run_number)

        return {'method': label, 'resolution': resolution, 'run': run_number, 'rmse': rmse, 'mae': mae, 't_grid': t_grid, 'gt_g': gt_g, 'm_g': m_g, 'steps': steps}
    except Exception as e:
        print(f"Error processing {bag_path}: {e}")
        return None

def plot_bar(df, metric):
    plt.figure(figsize=(12, 6))
    for resolution in df['resolution'].unique():
        sub_df = df[df['resolution'] == resolution].sort_values('method')
        methods = sub_df['method']
        values = sub_df[metric]
        plt.bar(methods, values, label=f'{resolution} {metric.upper()}')

    plt.xlabel('Method')
    plt.ylabel(metric.upper())
    plt.title(f'Combined {metric.upper()} for Both Resolutions')
    plt.xticks(rotation=45, ha='right')
    plt.legend()
    plt.tight_layout()
    filename = f'combined_{metric}.png'
    print(f"Saving combined bar plot to {filename}")
    plt.savefig(filename)
    plt.close()

def main():
    # Method configuration: directory name -> (label, topic)
    method_config = {
        "LFN3": ("LiteFlowNet3", "/optical_flow/LFN3_velocity"),
        "PWC_Net": ("PWC-Net", "/optical_flow/PWC_velocity"),
        "RAFT_Large": ("RAFT-Large", "/optical_flow/raft_large_velocity"),
        "RAFT_Small": ("RAFT-Small", "/optical_flow/raft_small_velocity"),
        "Lucas_Kanade_Heavy": ("LK_V3", "/optical_flow/LK_velocity"),
        "Lucas_Kanade_Light": ("Lukas Kanade", "/optical_flow/LK_velocity"),
        "Lucas_Kanade_Medium": ("LK_V2", "/optical_flow/LK_velocity"),
    }

    # Base directories for both resolutions
    base_dir_960 = "/home/docker/OpticalFlowSlider/ros2_ws/my_rosbag/Experiment1/Results_960"
    base_dir_640 = "/home/docker/OpticalFlowSlider/ros2_ws/my_rosbag/Experiment1/Results_640"  # Assuming this is the path for 640x480

    results = []

    # Process bags for both resolutions
    for base_dir, resolution in [(base_dir_640, "640x480"), (base_dir_960, "960x540")]:
        bag_paths = glob.glob(os.path.join(base_dir, "**", "*.db3"), recursive=True)
        if not bag_paths:
            print(f"No .db3 files found under {base_dir}")
            continue

        for bag_path in bag_paths:
            method_dir = os.path.basename(os.path.dirname(os.path.dirname(bag_path)))
            if method_dir not in method_config:
                print(f"Unknown method directory: {method_dir}")
                continue
            result = process_bag(bag_path, method_dir, resolution, method_config)
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
    avg_df.to_csv("metrics_average.csv", index=False)
    print("\n### Average Metrics")
    print(avg_df.to_string(index=False))

    # Generate combined bar plots for RMSE and MAE
    plot_bar(avg_df, 'rmse')
    plot_bar(avg_df, 'mae')

    # Generate combined aligned plots for each method and run
    methods = df['method'].unique()
    runs = df['run'].unique()
    for method in methods:
        for run in runs:
            data_640 = df[(df['method'] == method) & (df['run'] == run) & (df['resolution'] == '640x480')]
            data_960 = df[(df['method'] == method) & (df['run'] == run) & (df['resolution'] == '960x540')]
            if not data_640.empty and not data_960.empty:
                data_640 = data_640.iloc[0]
                data_960 = data_960.iloc[0]
                plot_combined_aligned_data(
                    data_640['t_grid'], data_640['gt_g'], data_640['m_g'], data_640['steps'],
                    data_960['t_grid'], data_960['gt_g'], data_960['m_g'], data_960['steps'],
                    method, run
                )

if __name__ == "__main__":
    main()