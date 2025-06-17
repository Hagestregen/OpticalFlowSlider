# #!/usr/bin/env python3
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
# from std_msgs.msg import Float64
# from scipy.interpolate import interp1d
# from scipy.signal import correlate
# import argparse

# # --- Helper Functions ---

# def extract_velocity_data(bag_path, gt_topic, meas_topics):
#     reader = SequentialReader()
#     storage = StorageOptions(uri=bag_path, storage_id='sqlite3')
#     conv = ConverterOptions('', '')
#     reader.open(storage, conv)

#     gt_times, gt_vals = [], []
#     meas_data = {topic: ([], []) for topic in meas_topics}

#     while reader.has_next():
#         topic, data, t = reader.read_next()
#         ts = t / 1e9
#         if topic == gt_topic:
#             msg = deserialize_message(data, TwistStamped)
#             gt_times.append(ts)
#             gt_vals.append(msg.twist.linear.x)
#         elif topic in meas_topics:
#             if topic == "/kalman_filter/imu_filtered":
#                 msg = deserialize_message(data, Float64)
#                 meas_data[topic][0].append(ts)
#                 meas_data[topic][1].append(msg.data)
#             else:
#                 msg = deserialize_message(data, Vector3Stamped)
#                 meas_data[topic][0].append(ts)
#                 if topic == "/kalman_filter/state":
#                     meas_data[topic][1].append(msg.vector.y)
#                 else:
#                     meas_data[topic][1].append(msg.vector.x)

#     gt = (np.array(gt_times), np.array(gt_vals))
#     meas = {t: (np.array(times), np.array(vals)) for t, (times, vals) in meas_data.items()}
#     return gt, meas


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
#     return int(round(tau / dt)), tau


# def compute_rmse(a, b):
#     n = min(len(a), len(b))
#     return np.sqrt(np.mean((a[:n] - b[:n])**2))


# def compute_mae(a, b):
#     n = min(len(a), len(b))
#     return np.mean(np.abs(a[:n] - b[:n]))


# def plot_aligned_data(tg, gtg, mg, steps, method, topic_name, resolution, run, out_dir, y_lim=None):
#     if steps > 0:
#         m_al, g_al = mg[:-steps], gtg[steps:]
#     elif steps < 0:
#         m_al, g_al = mg[-steps:], gtg[:steps]
#     else:
#         m_al, g_al = mg, gtg

#     if len(m_al) == 0 or len(g_al) == 0:
#         print(f"  → skip plotting {topic_name} for run {run}: no data after shift")
#         return

#     t = tg[:len(m_al)] - tg[0]
#     color = {"IMU Filtered Velocity": "orange", "KF Velocity": "green"}.get(topic_name, "blue")

#     plt.figure(figsize=(12, 6))
#     plt.plot(t, g_al, 'k', label='Motor Velocity')
#     plt.plot(t, m_al, color, label=topic_name)
#     plt.xlabel('Time (s)')
#     plt.ylabel('Velocity')
#     plt.title(f'{topic_name} vs Motor Velocity')
#     if y_lim:
#         plt.ylim(y_lim)
#     plt.legend()
#     plt.tight_layout()

#     fn = f'plot_{method}_{topic_name}_{resolution.replace("x", "_")}_run{run}.png'
#     path = os.path.join(out_dir, fn)
#     plt.savefig(path)
#     plt.close()
#     print(f"Saved aligned plot: {path}")


# def plot_error(tg, gtg, mg, method, resolution, run, out_dir):
#     error = gtg - mg
#     t = tg - tg[0]

#     plt.figure(figsize=(12, 6))
#     plt.plot(t, error, label='Error (GT – KF)')
#     plt.axhline(0, color='gray', linestyle='--', label='Zero Error')
#     plt.xlabel('Time (s)')
#     plt.ylabel('Velocity Error (m/s)')
#     plt.title('Error Plot: KF vs Motor Velocity')
#     plt.ylim(-0.55, 0.55)
#     plt.legend()
#     plt.grid(True)
#     plt.tight_layout()

#     fn = f'error_plot_{method}_{resolution.replace("x", "_")}_run{run}.png'
#     path = os.path.join(out_dir, fn)
#     plt.savefig(path)
#     plt.close()
#     print(f"Saved error plot: {path}")


# def process_bag(bag_path, method, resolution, meas_topics, out_dir, y_lim=None, dt=0.01):
#     gt, meas = extract_velocity_data(bag_path, "/motor/present_velocity", meas_topics)
#     if len(gt[0]) < 2:
#         print(f"Skipping {bag_path}: insufficient GT points")
#         return []

#     results = []
#     run = os.path.basename(os.path.dirname(bag_path))

#     for topic in meas_topics:
#         times, vals = meas[topic]
#         if len(times) < 2:
#             print(f"  → skip {topic}: too few points")
#             continue

#         t0, t1 = max(gt[0][0], times[0]), min(gt[0][-1], times[-1])
#         # remove first second of data
#         t0 += 1.0
#         if t1 <= t0:
#             print(f"  → skip {topic}: no overlap")
#             continue

#         tg = np.arange(t0, t1, dt)
#         gtg = interpolate_to_grid(gt[0], gt[1], tg)
#         mg = interpolate_to_grid(times, vals, tg)

#         steps, _ = find_optimal_shift(gtg, mg, dt)
#         if len(tg) - abs(steps) <= 0:
#             print(f"  → skip {topic}: no overlapped samples after shift")
#             continue

#         topic_name = (
#             "KF Velocity" if topic == "/kalman_filter/state" else
#             "IMU Filtered Velocity" if topic == "/kalman_filter/imu_filtered" else
#             topic.split('/')[-1]
#         )

#         plot_aligned_data(tg, gtg, mg, steps, method, topic_name, resolution, run, out_dir, y_lim)
#         if topic == "/kalman_filter/state":
#             plot_error(tg, gtg, mg, method, resolution, run, out_dir)

#         if steps > 0:
#             a, b = gtg[steps:], mg[:-steps]
#         else:
#             a, b = gtg[:steps], mg[-steps:]

#         results.append({
#             'method': method,
#             'topic': topic_name,
#             'resolution': resolution,
#             'run': run,
#             'rmse': compute_rmse(a, b),
#             'mae': compute_mae(a, b)
#         })

#     return results


# def plot_bar(df, resolution, metric, out_dir, y_lim=None):
#     sub = df[df['resolution'] == resolution].copy()
#     sub['label'] = sub['method'] + ' - ' + sub['topic']
#     grp = sub.groupby('label')[metric].mean().sort_index().reset_index()

#     plt.figure(figsize=(12, 6))
#     bars = plt.bar(grp['label'], grp[metric])
#     plt.xlabel('Method - Topic')
#     plt.ylabel(metric.upper())
#     plt.title(f'Average {metric.upper()} for {resolution}')
#     if y_lim:
#         plt.ylim(y_lim)
#     plt.xticks(rotation=45, ha='right')
#     for bar in bars:
#         h = bar.get_height()
#         if np.isfinite(h):
#             plt.text(bar.get_x() + bar.get_width()/2, h, f'{h:.4f}', ha='center', va='bottom')
#     plt.tight_layout()

#     fn = f'{metric}_{resolution.replace("x", "_")}_per_topic.png'
#     path = os.path.join(out_dir, fn)
#     plt.savefig(path)
#     plt.close()
#     print(f"Saved bar plot: {path}")


# def plot_histogram_avg(avg_df, out_dir):
#     # Prepare mapping for method-specific short codes and desired order
#     short_codes = {
#         'Lukas Kanade': 'LK',
#         'LiteFlowNet3': 'LFN3',
#         'RAFT-Large': 'RAFT'
#     }
#     method_order = ['Lukas Kanade', 'LiteFlowNet3', 'RAFT-Large']

#     categories = []
#     rmse_vals = []
#     mae_vals = []

#     # For each method in specified order, add Kalman Filter and Only entries
#     for method in method_order:
#         code = short_codes[method]
#         kf = avg_df[(avg_df['method'] == method) & (avg_df['topic'] == 'KF Velocity')][['rmse','mae']].iloc[0]
#         categories.append(f'Kalman Filter {code}')
#         rmse_vals.append(kf['rmse'])
#         mae_vals.append(kf['mae'])
#         opt = avg_df[(avg_df['method'] == method) & (~avg_df['topic'].isin(['KF Velocity','IMU Filtered Velocity']))][['rmse','mae']].iloc[0]
#         categories.append(f'{code}-Only')
#         rmse_vals.append(opt['rmse'])
#         mae_vals.append(opt['mae'])

#     # Aggregate IMU across all methods (average)
#     imu_rows = avg_df[avg_df['topic'] == 'IMU Filtered Velocity'][['rmse','mae']]
#     imu = imu_rows.mean()
#     categories.append('IMU-Only')
#     rmse_vals.append(imu['rmse'])
#     mae_vals.append(imu['mae'])

#     x = np.arange(len(categories))
#     width = 0.35

#     plt.figure(figsize=(12,6))
#     bars_rmse = plt.bar(x - width/2, rmse_vals, width, label='RMSE', color='#1f77b4')
#     bars_mae  = plt.bar(x + width/2, mae_vals,  width, label='MAE',  color='#ff7f0e')

#     for bar in bars_rmse:
#         y = bar.get_height()
#         plt.text(bar.get_x() + bar.get_width()/2, y, f'{y:.4f}', ha='center', va='bottom')
#     for bar in bars_mae:
#         y = bar.get_height()
#         plt.text(bar.get_x() + bar.get_width()/2, y, f'{y:.4f}', ha='center', va='bottom')

#     plt.xticks(x, categories, rotation=45, ha='right')
#     plt.ylabel('Error')
#     plt.title('Average RMSE and MAE by Category')
#     plt.legend()
#     plt.tight_layout()

#     fn = 'histogram_avg_metrics.png'
#     path = os.path.join(out_dir, fn)
#     plt.savefig(path)
#     plt.close()
#     print(f"Saved average-metrics histogram: {path}")


# def main():
#     parser = argparse.ArgumentParser(
#         description="Process ROS 2 bag files for optical flow methods."
#     )
#     parser.add_argument('--base_dir',   type=str, required=True,
#                         help="Base directory containing the bag files.")
#     parser.add_argument('--methods',    type=str, nargs='+',
#                         help="Methods to process (e.g. LiteFlowNet3 Lukas_Kanade RAFT_Large).")
#     parser.add_argument('--runs',       type=str, nargs='+',
#                         help="Specific runs to include.")
#     parser.add_argument('--output_dir', type=str, default='KF_Mounted_2',
#                         help="Where to save all plots and CSVs.")
#     parser.add_argument('--resolution', type=str, default='640x480',
#                         help="Data resolution.")
#     parser.add_argument('--y_lim',      type=float, nargs=2,
#                         help="Optional y-axis limits.")
#     args = parser.parse_args()

#     alias_map = {"Lucas_Kanade": "Lukas_Kanade"}
#     if args.methods:
#         args.methods = [alias_map.get(m, m) for m in args.methods]

#     os.makedirs(args.output_dir, exist_ok=True)

#     method_config = {
#         "LiteFlowNet3": ("LiteFlowNet3", [
#             "/optical_flow/LFN3_velocity",
#             "/kalman_filter/imu_filtered",
#             "/kalman_filter/state"
#         ]),
#         "Lukas_Kanade": ("Lukas Kanade", [
#             "/optical_flow/LK_velocity",
#             "/kalman_filter/imu_filtered",
#             "/kalman_filter/state"
#         ]),
#         "RAFT_Large":   ("RAFT-Large", [
#             "/optical_flow/raft_large_velocity",
#             "/kalman_filter/imu_filtered",
#             "/kalman_filter/state"
#         ]),
#     }

#     if not args.methods:
#         dirs = [d for d in os.listdir(args.base_dir) if os.path.isdir(os.path.join(args.base_dir, d))]
#         args.methods = [m for m in dirs if m in method_config]
#         if not args.methods:
#             print(f"No valid methods found in {args.base_dir}.")
#             return

#     all_results = []
#     for m in args.methods:
#         if m not in method_config:
#             print(f"Skipping unknown method '{m}'.")
#             continue
#         label, topics = method_config[m]
#         method_dir = os.path.join(args.base_dir, m)
#         if not os.path.isdir(method_dir):
#             print(f"Directory not found: {method_dir}")
#             continue

#         bags = glob.glob(os.path.join(method_dir, "**", "*.db3"), recursive=True)
#         if not bags:
#             print(f"No .db3 files under {method_dir}")
#             continue

#         for bag in bags:
#             run_name = os.path.basename(os.path.dirname(bag))
#             if args.runs and run_name not in args.runs:
#                 continue
#             all_results.extend(process_bag(bag, label, args.resolution, topics, args.output_dir, y_lim=args.y_lim))

#     if not all_results:
#         print("No results collected.")
#         return

#     df = pd.DataFrame(all_results)
#     per_run_csv = os.path.join(args.output_dir, "metrics_per_run.csv")
#     df.to_csv(per_run_csv, index=False)
#     print(f"Wrote per-run metrics to {per_run_csv}")

#     avg = df.groupby(['method','topic','resolution'])[['rmse','mae']].mean().reset_index()
#     avg_csv = os.path.join(args.output_dir, "metrics_average.csv")
#     avg.to_csv(avg_csv, index=False)
#     print(f"Wrote average metrics to {avg_csv}")

#     for res in df['resolution'].unique():
#         plot_bar(df, res, 'rmse', args.output_dir, y_lim=args.y_lim)
#         plot_bar(df, res, 'mae',  args.output_dir, y_lim=args.y_lim)

#     plot_histogram_avg(avg, args.output_dir)

# if __name__ == "__main__":
#     main()

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
from std_msgs.msg import Float64
from scipy.interpolate import interp1d
from scipy.signal import correlate
import argparse

# --- Helper Functions ---

def extract_velocity_data(bag_path, gt_topic, meas_topics):
    reader = SequentialReader()
    storage = StorageOptions(uri=bag_path, storage_id='sqlite3')
    conv = ConverterOptions('', '')
    reader.open(storage, conv)

    gt_times, gt_vals = [], []
    meas_data = {topic: ([], []) for topic in meas_topics}

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
            else:
                msg = deserialize_message(data, Vector3Stamped)
                meas_data[topic][0].append(ts)
                if topic == "/kalman_filter/state":
                    meas_data[topic][1].append(msg.vector.y)
                else:
                    meas_data[topic][1].append(msg.vector.x)

    gt = (np.array(gt_times), np.array(gt_vals))
    meas = {t: (np.array(times), np.array(vals)) for t, (times, vals) in meas_data.items()}
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
    return int(round(tau / dt)), tau


def compute_rmse(a, b):
    n = min(len(a), len(b))
    return np.sqrt(np.mean((a[:n] - b[:n])**2))


def compute_mae(a, b):
    n = min(len(a), len(b))
    return np.mean(np.abs(a[:n] - b[:n]))


def plot_aligned_data(tg, gtg, mg, steps, method, topic_name, resolution, run, out_dir, y_lim=None):
    if steps > 0:
        m_al, g_al = mg[:-steps], gtg[steps:]
    elif steps < 0:
        m_al, g_al = mg[-steps:], gtg[:steps]
    else:
        m_al, g_al = mg, gtg

    if len(m_al) == 0 or len(g_al) == 0:
        print(f"  → skip plotting {topic_name} for run {run}: no data after shift")
        return

    t = tg[:len(m_al)] - tg[0]
    color = {"IMU Filtered Velocity": "orange", "KF Velocity": "green"}.get(topic_name, "blue")

    plt.figure(figsize=(12, 6))
    plt.plot(t, g_al, 'k', label='Motor Velocity')
    plt.plot(t, m_al, color, label=topic_name)
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity')
    plt.title(f'{topic_name} vs Motor Velocity')
    if y_lim:
        plt.ylim(y_lim)
    plt.legend()
    plt.tight_layout()

    fn = f'plot_{method}_{topic_name}_{resolution.replace("x", "_")}_run{run}.png'
    path = os.path.join(out_dir, fn)
    plt.savefig(path)
    plt.close()
    print(f"Saved aligned plot: {path}")


def plot_error(tg, gtg, mg, method, resolution, run, out_dir):
    error = gtg - mg
    t = tg - tg[0]

    plt.figure(figsize=(12, 6))
    plt.plot(t, error, label='Error (GT – KF)')
    plt.axhline(0, color='gray', linestyle='--', label='Zero Error')
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity Error (m/s)')
    plt.title('Error Plot: KF vs Motor Velocity')
    plt.ylim(-0.55, 0.55)
    # show detailed y-axis ticks at 0.1 intervals
    plt.yticks(np.arange(-0.5, 0.51, 0.1))
    plt.legend()
    plt.grid(True)
    plt.tight_layout()

    fn = f'error_plot_{method}_{resolution.replace("x", "_")}_run{run}.png'
    path = os.path.join(out_dir, fn)
    plt.savefig(path)
    plt.close()
    print(f"Saved error plot: {path}")


def process_bag(bag_path, method, resolution, meas_topics, out_dir, y_lim=None, dt=0.01):
    gt, meas = extract_velocity_data(bag_path, "/motor/present_velocity", meas_topics)
    if len(gt[0]) < 2:
        print(f"Skipping {bag_path}: insufficient GT points")
        return []

    results = []
    run = os.path.basename(os.path.dirname(bag_path))

    for topic in meas_topics:
        times, vals = meas[topic]
        if len(times) < 2:
            print(f"  → skip {topic}: too few points")
            continue

        t0, t1 = max(gt[0][0], times[0]), min(gt[0][-1], times[-1])
        # remove first second of data
        t0 += 1.0
        if t1 <= t0:
            print(f"  → skip {topic}: no overlap")
            continue

        tg = np.arange(t0, t1, dt)
        gtg = interpolate_to_grid(gt[0], gt[1], tg)
        mg = interpolate_to_grid(times, vals, tg)

        steps, _ = find_optimal_shift(gtg, mg, dt)
        if len(tg) - abs(steps) <= 0:
            print(f"  → skip {topic}: no overlapped samples after shift")
            continue

        topic_name = (
            "KF Velocity" if topic == "/kalman_filter/state" else
            "IMU Filtered Velocity" if topic == "/kalman_filter/imu_filtered" else
            topic.split('/')[-1]
        )

        plot_aligned_data(tg, gtg, mg, steps, method, topic_name, resolution, run, out_dir, y_lim)
        if topic == "/kalman_filter/state":
            plot_error(tg, gtg, mg, method, resolution, run, out_dir)

        if steps > 0:
            a, b = gtg[steps:], mg[:-steps]
        else:
            a, b = gtg[:steps], mg[-steps:]

        results.append({
            'method': method,
            'topic': topic_name,
            'resolution': resolution,
            'run': run,
            'rmse': compute_rmse(a, b),
            'mae': compute_mae(a, b)
        })

    return results


def plot_bar(df, resolution, metric, out_dir, y_lim=None):
    sub = df[df['resolution'] == resolution].copy()
    sub['label'] = sub['method'] + ' - ' + sub['topic']
    grp = sub.groupby('label')[metric].mean().sort_index().reset_index()

    plt.figure(figsize=(12, 6))
    bars = plt.bar(grp['label'], grp[metric])
    plt.xlabel('Method - Topic')
    plt.ylabel(metric.upper())
    plt.title(f'Average {metric.upper()} for {resolution}')
    if y_lim:
        plt.ylim(y_lim)
    plt.xticks(rotation=45, ha='right')
    for bar in bars:
        h = bar.get_height()
        if np.isfinite(h):
            plt.text(bar.get_x() + bar.get_width()/2, h, f'{h:.4f}', ha='center', va='bottom')
    plt.tight_layout()

    fn = f'{metric}_{resolution.replace("x", "_")}_per_topic.png'
    path = os.path.join(out_dir, fn)
    plt.savefig(path)
    plt.close()
    print(f"Saved bar plot: {path}")


def plot_histogram_avg(avg_df, out_dir):
    # Prepare mapping for method-specific short codes and desired order
    short_codes = {
        'Lukas Kanade': 'LK',
        'LiteFlowNet3': 'LFN3',
        'RAFT-Large': 'RAFT'
    }
    method_order = ['Lukas Kanade', 'LiteFlowNet3', 'RAFT-Large']

    categories = []
    rmse_vals = []
    mae_vals = []

    # For each method in specified order, add Kalman Filter and Only entries
    for method in method_order:
        code = short_codes[method]
        kf = avg_df[(avg_df['method'] == method) & (avg_df['topic'] == 'KF Velocity')][['rmse','mae']].iloc[0]
        categories.append(f'Kalman Filter {code}')
        rmse_vals.append(kf['rmse'])
        mae_vals.append(kf['mae'])
        opt = avg_df[(avg_df['method'] == method) & (~avg_df['topic'].isin(['KF Velocity','IMU Filtered Velocity']))][['rmse','mae']].iloc[0]
        categories.append(f'{code}-Only')
        rmse_vals.append(opt['rmse'])
        mae_vals.append(opt['mae'])

    # Aggregate IMU across all methods (average)
    imu_rows = avg_df[avg_df['topic'] == 'IMU Filtered Velocity'][['rmse','mae']]
    imu = imu_rows.mean()
    categories.append('IMU-Only')
    rmse_vals.append(imu['rmse'])
    mae_vals.append(imu['mae'])

    x = np.arange(len(categories))
    width = 0.35

    plt.figure(figsize=(12,6))
    bars_rmse = plt.bar(x - width/2, rmse_vals, width, label='RMSE', color='#1f77b4')
    bars_mae  = plt.bar(x + width/2, mae_vals,  width, label='MAE',  color='#ff7f0e')

    for bar in bars_rmse:
        y = bar.get_height()
        plt.text(bar.get_x() + bar.get_width()/2, y, f'{y:.4f}', ha='center', va='bottom')
    for bar in bars_mae:
        y = bar.get_height()
        plt.text(bar.get_x() + bar.get_width()/2, y, f'{y:.4f}', ha='center', va='bottom')

    plt.xticks(x, categories, rotation=45, ha='right')
    plt.ylabel('Error')
    plt.title('Average RMSE and MAE by Category')
    plt.legend()
    plt.tight_layout()

    fn = 'histogram_avg_metrics.png'
    path = os.path.join(out_dir, fn)
    plt.savefig(path)
    plt.close()
    print(f"Saved average-metrics histogram: {path}")


def main():
    parser = argparse.ArgumentParser(
        description="Process ROS 2 bag files for optical flow methods."
    )
    parser.add_argument('--base_dir',   type=str, required=True,
                        help="Base directory containing the bag files.")
    parser.add_argument('--methods',    type=str, nargs='+',
                        help="Methods to process (e.g. LiteFlowNet3 Lukas_Kanade RAFT_Large).")
    parser.add_argument('--runs',       type=str, nargs='+',
                        help="Specific runs to include.")
    parser.add_argument('--output_dir', type=str, default='KF_GoodLight_2',
                        help="Where to save all plots and CSVs.")
    parser.add_argument('--resolution', type=str, default='640x480',
                        help="Data resolution.")
    parser.add_argument('--y_lim',      type=float, nargs=2,
                        help="Optional y-axis limits.")
    args = parser.parse_args()

    alias_map = {"Lucas_Kanade": "Lukas_Kanade"}
    if args.methods:
        args.methods = [alias_map.get(m, m) for m in args.methods]

    os.makedirs(args.output_dir, exist_ok=True)

    method_config = {
        "LiteFlowNet3": ("LiteFlowNet3", [
            "/optical_flow/LFN3_velocity",
            "/kalman_filter/imu_filtered",
            "/kalman_filter/state"
        ]),
        "Lukas_Kanade": ("Lukas Kanade", [
            "/optical_flow/LK_velocity",
            "/kalman_filter/imu_filtered",
            "/kalman_filter/state"
        ]),
        "RAFT_Large":   ("RAFT-Large", [
            "/optical_flow/raft_large_velocity",
            "/kalman_filter/imu_filtered",
            "/kalman_filter/state"
        ]),
    }

    if not args.methods:
        dirs = [d for d in os.listdir(args.base_dir) if os.path.isdir(os.path.join(args.base_dir, d))]
        args.methods = [m for m in dirs if m in method_config]
        if not args.methods:
            print(f"No valid methods found in {args.base_dir}.")
            return

    all_results = []
    for m in args.methods:
        if m not in method_config:
            print(f"Skipping unknown method '{m}'.")
            continue
        label, topics = method_config[m]
        method_dir = os.path.join(args.base_dir, m)
        if not os.path.isdir(method_dir):
            print(f"Directory not found: {method_dir}")
            continue

        bags = glob.glob(os.path.join(method_dir, "**", "*.db3"), recursive=True)
        if not bags:
            print(f"No .db3 files under {method_dir}")
            continue

        for bag in bags:
            run_name = os.path.basename(os.path.dirname(bag))
            if args.runs and run_name not in args.runs:
                continue
            all_results.extend(process_bag(bag, label, args.resolution, topics, args.output_dir, y_lim=args.y_lim))

    if not all_results:
        print("No results collected.")
        return

    df = pd.DataFrame(all_results)
    per_run_csv = os.path.join(args.output_dir, "metrics_per_run.csv")
    df.to_csv(per_run_csv, index=False)
    print(f"Wrote per-run metrics to {per_run_csv}")

    avg = df.groupby(['method','topic','resolution'])[['rmse','mae']].mean().reset_index()
    avg_csv = os.path.join(args.output_dir, "metrics_average.csv")
    avg.to_csv(avg_csv, index=False)
    print(f"Wrote average metrics to {avg_csv}")

    for res in df['resolution'].unique():
        plot_bar(df, res, 'rmse', args.output_dir, y_lim=args.y_lim)
        plot_bar(df, res, 'mae',  args.output_dir, y_lim=args.y_lim)

    plot_histogram_avg(avg, args.output_dir)

if __name__ == "__main__":
    main()
