#!/usr/bin/env python3
import os
import glob
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
method_order = ["Lukas Kanade Adapt", "Lukas Kanade", "PWC-Net", "LiteFlowNet3", "RAFT-Small", "RAFT-Large"]
# method_order = ["Lukas Kanade", "PWC-Net", "LiteFlowNet3", "RAFT-Small", "RAFT-Large"]
# Global output directory
output_dir = "."

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
    return np.sqrt(np.mean((a[:n] - b[:n])**2))

def compute_mae(a, b):
    n = min(len(a), len(b))
    return np.mean(np.abs(a[:n] - b[:n]))

def plot_aligned_data(t_grid, gt_g, m_g, steps, label, resolution, run, ylim=None):
    global output_dir
    if steps > 0:
        t_m = t_grid[:-steps]; m_al = m_g[:-steps]; g_al = gt_g[steps:]
    elif steps < 0:
        t_m = t_grid[-steps:]; m_al = m_g[-steps:]; g_al = gt_g[:steps]
    else:  # steps == 0
        t_m = t_grid; m_al = m_g; g_al = gt_g
    t_m_shifted = t_m - t_m[0]

    plt.figure(figsize=(12,6))
    plt.plot(t_m_shifted, g_al, label='Motor Velocity', color='black')
    plt.plot(t_m_shifted, m_al, label=f'{label} Flow (Shifted)', color='blue')
    plt.xlabel('Time (s)'); plt.ylabel('Velocity')
    plt.title(f'{label} Flow vs Motor Velocity ({resolution}, {run})')  # e.g., "Run 1"
    if ylim is not None: plt.ylim(ylim)
    plt.legend(); plt.tight_layout()

    fname = f"plot_{label.replace(' ', '_')}_{resolution.replace('x','_')}_{run}.png"  # e.g., "plot_Lukas_Kanade_640_480_run1.png"
    path = os.path.join(output_dir, fname)
    print(f"Saving plot to {path}")
    plt.savefig(path); plt.close()

def process_bag(bag_path, label, meas_topic, resolution, run_number, dt=0.01, ylim=None):
    """Process a single bag file and compute alignment metrics, with debug checks."""
    try:
        # 1) extract raw timestamp/value arrays
        (gt_t, gt_v), (m_t, m_v) = extract_velocity_data(
            bag_path, "/motor/present_velocity", meas_topic)

        print(f"Processing {bag_path}")
        print(f"  Ground-truth messages: {len(gt_t)};   Flow messages: {len(m_t)}")

        if len(gt_t) < 2 or len(m_t) < 2:
            raise ValueError("Insufficient data points")

        # 2) compute overlap window
        t0, t1 = max(gt_t[0], m_t[0]), min(gt_t[-1], m_t[-1])
        print(f"  Overlap window: t0={t0:.3f}, t1={t1:.3f},   duration={t1-t0:.3f}s")

        if t1 <= t0:
            raise ValueError("No time overlap between GT and flow signals")

        # 3) interpolate onto common grid
        t_grid = np.arange(t0, t1, dt)
        gt_g   = interpolate_to_grid(gt_t, gt_v, t_grid)
        m_g    = interpolate_to_grid(m_t, m_v, t_grid)

        if gt_g.size == 0 or m_g.size == 0:
            raise ValueError(f"Empty interpolation result: gt_g={gt_g.size}, m_g={m_g.size}")

        # 4) align via cross-correlation
        steps, tau = find_optimal_shift(gt_g, m_g, dt)
        if steps > 0:
            m_al = m_g[:-steps];   g_al = gt_g[steps:]
        elif steps < 0:
            m_al = m_g[-steps:];   g_al = gt_g[:steps]
        else:  # steps == 0
            m_al = m_g;   g_al = gt_g

        # 5) compute errors
        rmse = compute_rmse(g_al, m_al)
        mae  = compute_mae(g_al, m_al)

        # 6) plot and return
        plot_aligned_data(t_grid, gt_g, m_g, steps, label, resolution, run_number, ylim=ylim)
        return {'method': label, 'resolution': resolution,
                'run': run_number, 'rmse': rmse, 'mae': mae}

    except Exception as e:
        print(f"Error processing {bag_path}: {e}")
        return None

def plot_bar(df, resolution, metric, method_order):
    global output_dir
    sub_df = df[df['resolution'] == resolution].copy()
    sub_df['method'] = pd.Categorical(sub_df['method'], categories=method_order, ordered=True)
    sub_df = sub_df.sort_values('method')

    plt.figure(figsize=(12,6))
    # Use integer positions for bars
    bars = plt.bar(range(len(sub_df)), sub_df[metric])
    # Set x-tick labels to method names
    plt.xticks(range(len(sub_df)), sub_df['method'], rotation=45, ha='right')
    plt.xlabel('Method')
    plt.ylabel(metric.upper())
    plt.title(f'{metric.upper()} for {resolution}')
    
    # Annotate each bar
    for i, bar in enumerate(bars):
        yval = bar.get_height()
        plt.text(i, yval, f'{yval:.4f}', ha='center', va='bottom')

    plt.tight_layout()
    fname = f"{metric}_{resolution.replace('x','_')}.png"
    path = os.path.join(output_dir, fname)
    print(f"Saving bar plot to {path}")
    plt.savefig(path)
    plt.close()

def plot_combined(avg_df, resolution):
    """Plot RMSE and MAE side by side with labels."""
    global output_dir
    sub = avg_df[avg_df['resolution'] == resolution].copy()
    sub['method'] = pd.Categorical(sub['method'], categories=method_order, ordered=True)
    sub = sub.sort_values('method')
    methods = sub['method']
    x = np.arange(len(methods))
    width = 0.35

    plt.figure(figsize=(12,6))
    rmse_bars = plt.bar(x - width/2, sub['rmse'], width, label='RMSE', color='#1f77b4' )
    mae_bars  = plt.bar(x + width/2, sub['mae'],  width, label='MAE',  color="#ff7f0e")
    plt.xticks(x, methods, rotation=45, ha='right')
    plt.xlabel('Method'); plt.ylabel('Error')
    plt.title(f'RMSE vs MAE for {resolution}')
    plt.legend()

    # annotate combined bars
    for bar in rmse_bars:
        y = bar.get_height()
        plt.text(bar.get_x() + bar.get_width()/2, y, f'{y:.4f}', ha='center', va='bottom')
    for bar in mae_bars:
        y = bar.get_height()
        plt.text(bar.get_x() + bar.get_width()/2, y, f'{y:.4f}', ha='center', va='bottom')

    plt.tight_layout()
    fname = f"combined_{resolution.replace('x','_')}.png"
    path = os.path.join(output_dir, fname)
    print(f"Saving combined plot to {path}")
    plt.savefig(path); plt.close()

def main():
    global output_dir
    parser = argparse.ArgumentParser(description="Process ROS 2 bag files and analyze optical flow methods.")
    parser.add_argument('--base_dir',   type=str, required=True, help="Base dir containing ROS2 bags or parent folder")
    parser.add_argument('--resolution', type=str, required=True, help="Resolution, e.g., 640x480")
    parser.add_argument('--prefix',     type=str, default="results", help="Output folder name")
    parser.add_argument('--ylim',       type=float, nargs=2, default=None, help="Y-axis limits for aligned plots")
    args = parser.parse_args()

    output_dir = args.prefix
    os.makedirs(output_dir, exist_ok=True)

    # Updated method_config with additional Lucas Kanade variants
    # method_config = {
    #     "Lucas_Kanade": ("Lukas Kanade", "/optical_flow/LK_velocity"),
    #     "Lucas_Kanade_Adaptive": ("Lukas Kanade Adapt", "/optical_flow/LK_velocity"),
    #     # "Lucas_Kanade_Medium": ("Lukas Kanade Medium", "/optical_flow/LK_velocity"),
    #     # "Lucas_Kanade_Heavy": ("Lukas Kanade Heavy", "/optical_flow/LK_velocity"),
    #     "LFN3": ("LiteFlowNet3", "/optical_flow/LFN3_velocity"),
    #     "PWC_Net": ("PWC-Net", "/optical_flow/PWC_velocity"),
    #     "RAFT_Large": ("RAFT-Large", "/optical_flow/raft_large_velocity"),
    #     "RAFT_Small": ("RAFT-Small", "/optical_flow/raft_small_velocity"),
    # }
    method_config = {
        "Lucas_Kanade": ("Lukas Kanade", "/inertialsense/velocity_no_bias"),
    }

    # Determine if base_dir is a single method or a parent folder
    if os.path.basename(args.base_dir) in method_config:
        print(f"Processing single method: {os.path.basename(args.base_dir)}")
        method_dirs = [args.base_dir]
    else:
        method_dirs = [os.path.join(args.base_dir, d) for d in os.listdir(args.base_dir)
                       if os.path.isdir(os.path.join(args.base_dir, d)) and d in method_config]
        if method_dirs:
            print(f"Processing multiple methods: {[os.path.basename(d) for d in method_dirs]}")
        else:
            print(f"No known method directories found under {args.base_dir}")
            return

    results = []
    for method_dir in method_dirs:
        method_name = os.path.basename(method_dir)
        if method_name not in method_config:
            print(f"Skipping unknown method directory: {method_name}")
            continue
        label, meas_topic = method_config[method_name]
        bag_paths = sorted(glob.glob(os.path.join(method_dir, "**", "*.db3"), recursive=True))
        if not bag_paths:
            print(f"No .db3 files found under {method_dir}")
            continue
        print(f"Found {len(bag_paths)} bag files for method {method_name}")
        for i, bag in enumerate(bag_paths, start=1):
            run_number = f"run{i}"
            r = process_bag(bag, label, meas_topic, args.resolution, run_number, ylim=args.ylim)
            if r:
                results.append(r)

    if not results:
        print("No results collected. Exiting.")
        return

    df = pd.DataFrame(results)
    df.to_csv(os.path.join(output_dir, "metrics_per_run.csv"), index=False)
    avg_df = df.groupby(['method', 'resolution']).mean(numeric_only=True).reset_index()[['method', 'resolution', 'rmse', 'mae']]
    avg_df.to_csv(os.path.join(output_dir, "metrics_average.csv"), index=False)

    for res in df['resolution'].unique():
        plot_bar(avg_df, res, 'rmse', method_order)
        plot_bar(avg_df, res, 'mae', method_order)
        plot_combined(avg_df, res)

if __name__ == "__main__":
    main()
# def main():
#     global output_dir
#     parser = argparse.ArgumentParser(description="Process ROS 2 bag files and analyze optical flow methods.")
#     parser.add_argument('--base_dir',   type=str, required=True, help="Base dir containing ROS2 bags")
#     parser.add_argument('--resolution', type=str, required=True, help="Resolution, e.g., 640x480")
#     parser.add_argument('--prefix',     type=str, default="results", help="Output folder name")
#     parser.add_argument('--ylim',       type=float, nargs=2, default=None, help="Y-axis limits for aligned plots")
#     args = parser.parse_args()

#     output_dir = args.prefix
#     os.makedirs(output_dir, exist_ok=True)

#     method_config = {
#         "Lucas_Kanade": ("Lukas Kanade", "/optical_flow/LK_velocity"),
#         "LFN3": ("LiteFlowNet3", "/optical_flow/LFN3_velocity"),
#         "PWC_Net": ("PWC-Net", "/optical_flow/PWC_velocity"),
#         "RAFT_Large": ("RAFT-Large", "/optical_flow/raft_large_velocity"),
#         "RAFT_Small": ("RAFT-Small", "/optical_flow/raft_small_velocity"),
#         # add other methods here...
#     }
    
#     method_dir = os.path.basename(args.base_dir)
#     if method_dir not in method_config:
#         print(f"Unknown method directory: {method_dir}")
#         return
#     label, meas_topic = method_config[method_dir]

#     # Get and sort bag paths to ensure consistent order
#     bag_paths = sorted(glob.glob(os.path.join(args.base_dir, "**", "*.db3"), recursive=True))
#     if not bag_paths:
#         print(f"No .db3 files found under {args.base_dir}")
#         return

#     results = []
#     # Process bags and assign run numbers sequentially
#     for i, bag in enumerate(bag_paths, start=1):
#         run_number = f"run{i}"  # Assign run1, run2, etc., based on order
#         r = process_bag(bag, label, meas_topic, args.resolution, run_number, ylim=args.ylim)
#         if r:
#             results.append(r)

#     if not results:
#         print("No results collected. Exiting.")
#         return

#     df = pd.DataFrame(results)
#     df.to_csv(os.path.join(output_dir, "metrics_per_run.csv"), index=False)
#     avg_df = df.groupby(['method','resolution']).mean(numeric_only=True).reset_index()[['method','resolution','rmse','mae']]
#     avg_df.to_csv(os.path.join(output_dir, "metrics_average.csv"), index=False)

#     for res in df['resolution'].unique():
#         plot_bar(avg_df, res, 'rmse', method_order)
#         plot_bar(avg_df, res, 'mae',  method_order)
#         plot_combined(avg_df, res)

# if __name__ == "__main__":
#     main()


# import os
# import glob
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
# import argparse

# # Define the desired order of methods for bar plots
# method_order = ["Lukas Kanade", "PWC-Net", "LiteFlowNet3", "RAFT-Small", "RAFT-Large"]

# # Global output directory
# output_dir = "."

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
#     return np.sqrt(np.mean((a[:n] - b[:n])**2))

# def compute_mae(a, b):
#     n = min(len(a), len(b))
#     return np.mean(np.abs(a[:n] - b[:n]))

# def plot_aligned_data(t_grid, gt_g, m_g, steps, label, resolution, run, ylim=None):
#     global output_dir
#     # if steps > 0:
#     #     t_m = t_grid[:-steps]; m_al = m_g[:-steps]; g_al = gt_g[steps:]
#     # else:
#     #     t_m = t_grid[-steps:]; m_al = m_g[-steps:]; g_al = gt_g[:steps]
#     # t_m_shifted = t_m - t_m[0]
#     if steps > 0:
#         t_m = t_grid[:-steps]; m_al = m_g[:-steps]; g_al = gt_g[steps:]
#     elif steps < 0:
#         t_m = t_grid[-steps:]; m_al = m_g[-steps:]; g_al = gt_g[:steps]
#     else:  # steps == 0
#         t_m = t_grid; m_al = m_g; g_al = gt_g
#     t_m_shifted = t_m - t_m[0]

#     plt.figure(figsize=(12,6))
#     plt.plot(t_m_shifted, g_al, label='Motor Velocity', color='black')
#     plt.plot(t_m_shifted, m_al, label=f'{label} Flow (Shifted)', color='blue')
#     plt.xlabel('Time (s)'); plt.ylabel('Velocity')
#     plt.title(f'{label} Flow vs Motor Velocity ({resolution}, {run})')
#     if ylim is not None: plt.ylim(ylim)
#     plt.legend(); plt.tight_layout()

#     fname = f"plot_{label.replace(' ', '_')}_{resolution.replace('x','_')}_run{run}.png"
#     path = os.path.join(output_dir, fname)
#     print(f"Saving plot to {path}")
#     plt.savefig(path); plt.close()

# def process_bag(bag_path, label, meas_topic, resolution, dt=0.01, ylim=None):
#     """Process a single bag file and compute alignment metrics, with debug checks."""
#     bag_dir = os.path.basename(os.path.dirname(bag_path))
#     parts = bag_dir.split('_')
#     run_number = parts[-1] if len(parts) > 0 else 'unknown'  # Fix: Use last part for run number

#     try:
#         # 1) extract raw timestamp/value arrays
#         (gt_t, gt_v), (m_t, m_v) = extract_velocity_data(
#             bag_path, "/motor/present_velocity", meas_topic)

#         print(f"Processing {bag_path}")
#         print(f"  Ground-truth messages: {len(gt_t)};   Flow messages: {len(m_t)}")

#         if len(gt_t) < 2 or len(m_t) < 2:
#             raise ValueError("Insufficient data points")

#         # 2) compute overlap window
#         t0, t1 = max(gt_t[0], m_t[0]), min(gt_t[-1], m_t[-1])
#         print(f"  Overlap window: t0={t0:.3f}, t1={t1:.3f},   duration={t1-t0:.3f}s")

#         if t1 <= t0:
#             raise ValueError("No time overlap between GT and flow signals")

#         # 3) interpolate onto common grid
#         t_grid = np.arange(t0, t1, dt)
#         gt_g   = interpolate_to_grid(gt_t, gt_v, t_grid)
#         m_g    = interpolate_to_grid(m_t, m_v, t_grid)

#         if gt_g.size == 0 or m_g.size == 0:
#             raise ValueError(f"Empty interpolation result: gt_g={gt_g.size}, m_g={m_g.size}")

#         # 4) align via cross-correlation
#         steps, tau = find_optimal_shift(gt_g, m_g, dt)
#         if steps > 0:
#             m_al = m_g[:-steps];   g_al = gt_g[steps:]
#         elif steps < 0:
#             m_al = m_g[-steps:];   g_al = gt_g[:steps]
#         else:  # steps == 0
#             m_al = m_g;   g_al = gt_g  # Fix: Handle steps == 0 correctly

#         # 5) compute errors
#         rmse = compute_rmse(g_al, m_al)
#         mae  = compute_mae(g_al, m_al)

#         # 6) plot and return
#         plot_aligned_data(t_grid, gt_g, m_g, steps, label, resolution, run_number, ylim=ylim)
#         return {'method': label, 'resolution': resolution,
#                 'run': run_number, 'rmse': rmse, 'mae': mae}

#     except Exception as e:
#         print(f"Error processing {bag_path}: {e}")
#         return None



# def plot_bar(df, resolution, metric, method_order):
#     global output_dir
#     sub_df = df[df['resolution'] == resolution].copy()
#     sub_df['method'] = pd.Categorical(sub_df['method'], categories=method_order, ordered=True)
#     sub_df = sub_df.sort_values('method')

#     plt.figure(figsize=(12,6))
#     bars = plt.bar(sub_df['method'], sub_df[metric])
#     plt.xlabel('Method'); plt.ylabel(metric.upper())
#     plt.title(f'{metric.upper()} for {resolution}')
#     plt.xticks(rotation=45, ha='right')

#     # annotate each bar
#     for bar in bars:
#         yval = bar.get_height()
#         plt.text(bar.get_x() + bar.get_width()/2, yval, f'{yval:.4f}', ha='center', va='bottom')

#     plt.tight_layout()
#     fname = f"{metric}_{resolution.replace('x','_')}.png"
#     path = os.path.join(output_dir, fname)
#     print(f"Saving bar plot to {path}")
#     plt.savefig(path); plt.close()

# def plot_combined(avg_df, resolution):
#     """Plot RMSE and MAE side by side with labels."""
#     global output_dir
#     sub = avg_df[avg_df['resolution'] == resolution].copy()
#     sub['method'] = pd.Categorical(sub['method'], categories=method_order, ordered=True)
#     sub = sub.sort_values('method')
#     methods = sub['method']
#     x = np.arange(len(methods))
#     width = 0.35

#     plt.figure(figsize=(12,6))
#     rmse_bars = plt.bar(x - width/2, sub['rmse'], width, label='RMSE', color='#1f77b4')
#     mae_bars  = plt.bar(x + width/2, sub['mae'],  width, label='MAE',  color='#ff7f0e')
#     plt.xticks(x, methods, rotation=45, ha='right')
#     plt.xlabel('Method'); plt.ylabel('Error')
#     plt.title(f'RMSE vs MAE for {resolution}')
#     plt.legend()

#     # annotate combined bars
#     for bar in rmse_bars:
#         y = bar.get_height()
#         plt.text(bar.get_x() + bar.get_width()/2, y, f'{y:.4f}', ha='center', va='bottom')
#     for bar in mae_bars:
#         y = bar.get_height()
#         plt.text(bar.get_x() + bar.get_width()/2, y, f'{y:.4f}', ha='center', va='bottom')

#     plt.tight_layout()
#     fname = f"combined_{resolution.replace('x','_')}.png"
#     path = os.path.join(output_dir, fname)
#     print(f"Saving combined plot to {path}")
#     plt.savefig(path); plt.close()

# def main():
#     global output_dir
#     parser = argparse.ArgumentParser(description="Process ROS 2 bag files and analyze optical flow methods.")
#     parser.add_argument('--base_dir',   type=str, required=True, help="Base dir containing ROS2 bags")
#     parser.add_argument('--resolution', type=str, required=True, help="Resolution, e.g., 640x480")
#     parser.add_argument('--prefix',     type=str, default="results", help="Output folder name")
#     parser.add_argument('--ylim',       type=float, nargs=2, default=None, help="Y‐axis limits for aligned plots")
#     args = parser.parse_args()

#     output_dir = args.prefix
#     os.makedirs(output_dir, exist_ok=True)

#     method_config = {
#         "Lucas_Kanade": ("Lukas Kanade", "/optical_flow/LK_smooth_velocity"),
#         # add other methods here...
#     }
#     method_dir = os.path.basename(args.base_dir)
#     if method_dir not in method_config:
#         print(f"Unknown method directory: {method_dir}")
#         return
#     label, meas_topic = method_config[method_dir]

#     bag_paths = glob.glob(os.path.join(args.base_dir, "**", "*.db3"), recursive=True)
#     if not bag_paths:
#         print(f"No .db3 files found under {args.base_dir}")
#         return

#     results = []
#     for bag in bag_paths:
#         r = process_bag(bag, label, meas_topic, args.resolution, ylim=args.ylim)
#         if r:
#             results.append(r)

#     if not results:
#         print("No results collected. Exiting.")
#         return

#     df = pd.DataFrame(results)
#     df.to_csv(os.path.join(output_dir, "metrics_per_run.csv"), index=False)
#     avg_df = df.groupby(['method','resolution']).mean(numeric_only=True).reset_index()[['method','resolution','rmse','mae']]
#     avg_df.to_csv(os.path.join(output_dir, "metrics_average.csv"), index=False)

#     for res in df['resolution'].unique():
#         plot_bar(avg_df, res, 'rmse', method_order)
#         plot_bar(avg_df, res, 'mae',  method_order)
#         plot_combined(avg_df, res)

# if __name__ == "__main__":
#     main()



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
# import argparse

# # Define the desired order of methods for bar plots
# method_order = ["Lukas Kanade", "PWC-Net", "LiteFlowNet3", "RAFT-Small", "RAFT-Large"]

# # --- Helper Functions ---

# def extract_velocity_data(bag_path, gt_topic, meas_topic):
#     """Extract velocity data from a ROS 2 bag file."""
#     reader = SequentialReader()
#     storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
#     converter_options = ConverterOptions('', '')
#     reader.open(storage_options, converter_options)

#     gt_times, gt_vals = [], []
#     meas_times, meas_vals = [], []

#     while reader.has_next():
#         topic, data, t = reader.read_next()
#         ts = t / 1e9  # Convert nanoseconds to seconds
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
#     """Interpolate data onto a uniform time grid."""
#     if len(times) < 2:
#         raise ValueError("Need at least 2 points for interpolation")
#     f = interp1d(times, values, bounds_error=False, fill_value="extrapolate")
#     return f(t_grid)

# def find_optimal_shift(gt_grid, m_grid, dt):
#     """Find the optimal time shift between two signals using cross-correlation."""
#     corr = correlate(gt_grid - gt_grid.mean(), m_grid - m_grid.mean(), mode='full')
#     lags = np.arange(-len(m_grid) + 1, len(gt_grid))
#     idx = np.argmax(corr)
#     tau = lags[idx] * dt
#     steps = int(round(tau / dt))
#     return steps, tau

# def compute_rmse(a, b):
#     """Compute the Root Mean Square Error between two arrays."""
#     n = min(len(a), len(b))
#     return np.sqrt(np.mean((a[:n] - b[:n]) ** 2))

# def compute_mae(a, b):
#     """Compute the Mean Absolute Error between two arrays."""
#     n = min(len(a), len(b))
#     return np.mean(np.abs(a[:n] - b[:n]))

# def plot_aligned_data(t_grid, gt_g, m_g, steps, label, resolution, run, ylim=None):
#     """Plot aligned ground truth and measured data with optional y-axis limits."""
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
#     plt.plot(t_m_shifted, m_al, label=f'{label} Flow (Shifted)', color='blue')
#     plt.xlabel('Time (s)')
#     plt.ylabel('Velocity')
#     plt.title(f'{label} Flow vs Motor Velocity ({resolution}, Run {run})')
#     if ylim is not None:
#         plt.ylim(ylim)  # Set y-axis limits if provided
#     plt.legend()
#     plt.tight_layout()
#     filename = f'plot_{label}_{resolution.replace("x", "_")}_run{run}.png'
#     print(f"Saving plot to {filename}")
#     plt.savefig(filename)
#     plt.close()

# def process_bag(bag_path, label, meas_topic, resolution, dt=0.01, ylim=None):
#     """Process a single bag file and compute alignment metrics."""
#     bag_dir = os.path.basename(os.path.dirname(bag_path))
#     parts = bag_dir.split('_')
#     run_number = parts[-2] if len(parts) > 1 else 'unknown'

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
#         plot_aligned_data(t_grid, gt_g, m_g, steps, label, resolution, run_number, ylim=ylim)

#         return {'method': label, 'resolution': resolution, 'run': run_number, 'rmse': rmse, 'mae': mae}
#     except Exception as e:
#         print(f"Error processing {bag_path}: {e}")
#         return None

# def plot_bar(df, resolution, metric, method_order):
#     """Generate a bar plot for a given metric and resolution."""
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
#     """Main function to process bag files and generate metrics and plots."""
#     parser = argparse.ArgumentParser(description="Process ROS 2 bag files and analyze optical flow methods.")
#     parser.add_argument('--base_dir', type=str, required=True, help="Base directory containing ROS 2 bag files")
#     parser.add_argument('--resolution', type=str, required=True, help="Resolution of the data, e.g., 640x480")
#     parser.add_argument('--ylim', type=float, nargs=2, default=None, 
#                         help="Set y-axis limits for velocity plots, e.g., --ylim -1.0 1.0 (min max)")
#     args = parser.parse_args()

#     # Method configuration: directory name -> (label, topic)
#     method_config = {
#         # "LFN3": ("LiteFlowNet3", "/optical_flow/LFN3_velocity"),
#         # "PWC_Net": ("PWC-Net", "/optical_flow/PWC_velocity"),
#         # "RAFT_Large_Bad_Light_640": ("RAFT-Large", "/optical_flow/raft_large_smooth_velocity"),
#         # "RAFT_Small": ("RAFT-Small", "/optical_flow/raft_small_velocity"),
#         "Lucas_Kanade_Light": ("Lukas Kanade", "/optical_flow/LK_smooth_velocity"),
#     }

#     # Extract method_dir from base_dir
#     method_dir = os.path.basename(args.base_dir)
#     if method_dir not in method_config:
#         print(f"Unknown method directory: {method_dir}")
#         return

#     label, meas_topic = method_config[method_dir]

#     results = []

#     # Process bags
#     bag_paths = glob.glob(os.path.join(args.base_dir, "**", "*.db3"), recursive=True)
#     if not bag_paths:
#         print(f"No .db3 files found under {args.base_dir}")
#         return

#     for bag_path in bag_paths:
#         result = process_bag(bag_path, label, meas_topic, args.resolution, ylim=args.ylim)
#         if result:
#             results.append(result)

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
