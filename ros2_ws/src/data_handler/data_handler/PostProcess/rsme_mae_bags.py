#!/usr/bin/env python3
import os
import glob
import csv
import rosbag2_py
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import Vector3Stamped, TwistStamped
from std_msgs.msg import Float64
import numpy as np
from scipy.interpolate import interp1d
from scipy.signal import correlate

# --- helper functions ---

def extract_velocity_data(bag_path, gt_topic, meas_topic, meas_type, field_path):
    reader = rosbag2_py.SequentialReader()
    storage_options   = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader.open(storage_options, converter_options)

    gt_times, gt_vals     = [], []
    meas_times, meas_vals = [], []

    def get_field(msg, path):
        obj = msg
        for p in path.split('.'):
            obj = getattr(obj, p)
        return obj

    while reader.has_next():
        topic, data, t = reader.read_next()
        ts = t / 1e9
        if topic == gt_topic:
            msg = deserialize_message(data, TwistStamped)
            gt_times.append(ts)
            gt_vals .append(msg.twist.linear.x)
        elif topic == meas_topic:
            msg = deserialize_message(data, meas_type)
            meas_times.append(ts)
            meas_vals .append(get_field(msg, field_path))

    return (np.array(gt_times),   np.array(gt_vals)), (np.array(meas_times), np.array(meas_vals))


def interpolate_to_grid(times, values, t_grid):
    if len(times) < 2:
        raise ValueError("Need ≥2 points")
    f = interp1d(times, values, bounds_error=False, fill_value="extrapolate")
    return f(t_grid)


def find_optimal_shift(gt_grid, m_grid, dt):
    corr = correlate(gt_grid - gt_grid.mean(), m_grid - m_grid.mean(), mode='full')
    lags = np.arange(-len(m_grid)+1, len(gt_grid))
    idx  = np.argmax(corr)
    tau  = lags[idx] * dt
    steps = int(round(tau / dt))
    return steps, tau


def compute_rmse(a, b):
    n = min(len(a), len(b))
    return np.sqrt(np.mean((a[:n] - b[:n])**2))


def compute_mae(a, b):
    n = min(len(a), len(b))
    return np.mean(np.abs(a[:n] - b[:n]))


# --- main ---
def main():
    # specify parent folder containing .db3 files
    parent_dir = "/home/docker/OpticalFlowSlider/ros2_ws/my_rosbag/Experiment2/LFN3/Combined_No_OOSM"
    bag_paths = sorted(glob.glob(os.path.join(parent_dir, "**", "*.db3"), recursive=True))
    out_csv = "metrics_No_OOSM.csv"
    avg_csv = "metrics_No_OOSM_avg.csv"

    # auto-discover runs
    if not bag_paths:
        raise RuntimeError(f"No .db3 files found under {parent_dir}")

    runs = [(f"run{i+1}", path) for i, path in enumerate(bag_paths)]

    gt_topic = "/motor/present_velocity"
    measurement_topics = [
        ("IMU (filtered)",   "/kalman_filter/imu_filtered",    Float64,        "data",      False),
        ("LFN3 flow",        "/optical_flow/LFN3_velocity",    Vector3Stamped, "vector.x",  True),
        ("LFN3 smooth",      "/optical_flow/LFN3_smooth_velocity", Vector3Stamped, "vector.x", True),
        ("KF state vel",     "/kalman_filter/state",           Vector3Stamped, "vector.y",  False),
    ]

    grid_freq = 100.0
    dt = 1.0 / grid_freq

    # open CSV for per-run metrics
    
    with open(out_csv, "w", newline="") as cf:
        w = csv.writer(cf)
        w.writerow(["run","topic","rmse","mae","shift_s","rmse_shift","mae_shift"])

        # console header
        hdr = f"{'run':<10} | {'topic':<15} | {'RMSE':>6} | {'MAE':>6} | {'shift(s)':>8} | {'RMSE_sh':>8} | {'MAE_sh':>8}"
        print(hdr)
        print('-'*len(hdr))

        # storage for averages
        per_topic = { name:{"rmse":[],"mae":[],"rmse_shift":[],"mae_shift":[],"taus":[]} for name,*_ in measurement_topics }

        for run_label, bag_path in runs:
            for name, topic, msg_cls, field, needs_shift in measurement_topics:
                try:
                    (gt_t, gt_v),(m_t,m_v)= extract_velocity_data(bag_path,gt_topic,topic,msg_cls,field)
                except Exception as e:
                    continue
                # time overlap
                t0 = max(gt_t[0], m_t[0])
                t1 = min(gt_t[-1],m_t[-1])
                if t1 <= t0: continue
                t_grid = np.arange(t0,t1,dt)
                gt_g, m_g = interpolate_to_grid(gt_t,gt_v,t_grid), interpolate_to_grid(m_t,m_v,t_grid)
                rmse_ns = compute_rmse(gt_g,m_g)
                mae_ns  = compute_mae (gt_g,m_g)
                shift_s = ''
                rmse_sh = ''
                mae_sh  = ''
                if needs_shift:
                    steps,tau = find_optimal_shift(gt_g,m_g,dt)
                    if steps>0:
                        m_al, g_al = m_g[:-steps], gt_g[steps:]
                    else:
                        m_al, g_al = m_g[-steps:], gt_g[:steps]
                    rmse_s = compute_rmse(g_al,m_al)
                    mae_s  = compute_mae (g_al,m_al)
                    shift_s = f"{tau:+.3f}"
                    rmse_sh = f"{rmse_s:.4f}"
                    mae_sh  = f"{mae_s:.4f}"
                    per_topic[name]["rmse_shift"].append(rmse_s)
                    per_topic[name]["mae_shift" ].append(mae_s)
                    per_topic[name]["taus"     ].append(tau)

                # record
                per_topic[name]["rmse"].append(rmse_ns)
                per_topic[name]["mae" ].append(mae_ns)

                # write row
                w.writerow([run_label,name,f"{rmse_ns:.4f}",f"{mae_ns:.4f}",shift_s,rmse_sh,mae_sh])
                print(f"{run_label:<10} | {name:<15} | {rmse_ns:>6.4f} | {mae_ns:>6.4f} | {shift_s:>8} | {rmse_sh:>8} | {mae_sh:>8}")

    # write averages to new CSV
    with open(avg_csv, "w", newline="") as af:
        w = csv.writer(af)
        w.writerow(["topic","avg_rmse","avg_mae","avg_shift","avg_rmse_shift","avg_mae_shift"])
        print("\n=== AVERAGE OVER ALL RUNS ===")
        for name,d in per_topic.items():
            a_rmse = np.mean(d["rmse"]) if d["rmse"] else np.nan
            a_mae  = np.mean(d["mae"])  if d["mae"]  else np.nan
            row=[name,f"{a_rmse:.4f}",f"{a_mae:.4f}"]
            if d["rmse_shift"]:
                a_tau   = np.mean(d["taus"])
                a_rmse_s= np.mean(d["rmse_shift"])
                a_mae_s = np.mean(d["mae_shift"])
                row += [f"{a_tau:+.3f}",f"{a_rmse_s:.4f}",f"{a_mae_s:.4f}"]
                print(f"{name:<15}  ⟨RMSE⟩={a_rmse:.4f}, ⟨MAE⟩={a_mae:.4f}, ⟨shift⟩={a_tau:+.3f}, ⟨RMSE_s⟩={a_rmse_s:.4f}, ⟨MAE_s⟩={a_mae_s:.4f}")
            else:
                row += ['', '', '']
                print(f"{name:<15}  ⟨RMSE⟩={a_rmse:.4f}, ⟨MAE⟩={a_mae:.4f}")
            w.writerow(row)

if __name__ == "__main__":
    main()



