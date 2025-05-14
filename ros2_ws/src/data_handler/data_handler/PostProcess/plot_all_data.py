import rosbag2_py
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import Vector3Stamped, TwistStamped
import numpy as np
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt

def extract_data(bag_path, topic, msg_type, field_path):
    """
    Extract timestamps and values from a ROS2 bag file for a specific topic.
    
    Args:
        bag_path (str): Path to the bag file.
        topic (str): Topic name to extract.
        msg_type: ROS2 message type (e.g., TwistStamped, Vector3Stamped).
        field_path (str): Dot-separated path to the desired field (e.g., 'twist.linear.x').
    
    Returns:
        tuple: (times, values) as numpy arrays.
    """
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader.open(storage_options, converter_options)
    
    times, values = [], []
    message_count = 0
    while reader.has_next():
        (topic_read, data, t) = reader.read_next()
        if topic_read == topic:
            msg = deserialize_message(data, msg_type)
            times.append(t / 1e9)  # Convert nanoseconds to seconds
            val = msg
            for attr in field_path.split('.'):
                val = getattr(val, attr)
            values.append(val)
            message_count += 1
            if message_count <= 5:  # Print first 5 messages for inspection
                print(f"Topic: {topic}, Time: {t / 1e9}, Value: {val}")
                print(f"Extracted {message_count} messages from {bag_path} for topic {topic}")
    
    return np.array(times), np.array(values)

def interpolate_to_grid(times, values, t_grid):
    """
    Interpolate values onto a specified time grid with NaN outside bounds.
    
    Args:
        times (np.ndarray): Original timestamps.
        values (np.ndarray): Original values.
        t_grid (np.ndarray): Target time grid.
    
    Returns:
        np.ndarray: Interpolated values with NaN outside original time range.
    """
    if len(times) < 2:
        raise ValueError("Need at least 2 points for interpolation")
    interp_func = interp1d(times, values, bounds_error=False, fill_value=np.nan)
    return interp_func(t_grid)

def compute_error(gt_grid, of_grid, k):
    """
    Compute RMSE between ground truth and shifted optical flow data for overlapping regions.
    
    Args:
        gt_grid (np.ndarray): Interpolated ground truth data.
        of_grid (np.ndarray): Interpolated optical flow data.
        k (int): Number of steps to shift of_grid (positive = right, negative = left).
    
    Returns:
        float: RMSE for overlapping valid data points, or inf if no overlap.
    """
    if k > 0:
        aligned_of = np.concatenate((of_grid[k:], np.full(k, np.nan)))
    elif k < 0:
        aligned_of = np.concatenate((np.full(-k, np.nan), of_grid[:k]))
    else:
        aligned_of = of_grid.copy()
    valid = np.isfinite(gt_grid) & np.isfinite(aligned_of)
    if np.sum(valid) == 0:
        return np.inf
    error = np.sqrt(np.mean((gt_grid[valid] - aligned_of[valid])**2))  # RMSE
    return error

def main():
    # Bag paths (verify 'motor' path)
    bag_paths = {
        'motor': '/home/docker/OpticalFlowSlider/ros2_ws/my_rosbag/Experiment1/recording/Experiment1_640/Experiment1_640_0.db3',
        'LFN3': '/home/docker/OpticalFlowSlider/ros2_ws/my_rosbag/Experiment1/flow/lfn3_640/Experiment1_flow_640_0_noKF.db3_0.db3',
        'PWC': '/home/docker/OpticalFlowSlider/ros2_ws/my_rosbag/Experiment1/flow/pwc_640/Experiment1_pwc_640_1_0.db3',
        'raft_small': '/home/docker/OpticalFlowSlider/ros2_ws/my_rosbag/Experiment1/flow/RAFT_640/Experiment1_RAFT_640_0.db3',
        'raft_large': '/home/docker/OpticalFlowSlider/ros2_ws/my_rosbag/Experiment1/flow/RAFT_L_640/Experiment1_RAFT_L_640_0.db3',
        'LK': '/home/docker/OpticalFlowSlider/ros2_ws/my_rosbag/Experiment1/flow/LK_640/Experiment1_LK_640_0.db3'
    }

    methods = [
        {'bag': bag_paths['LFN3'], 'topic': '/optical_flow/LFN3_velocity', 'label': 'LFN3'},
        {'bag': bag_paths['PWC'], 'topic': '/optical_flow/PWC_velocity', 'label': 'PWC-Net'},
        {'bag': bag_paths['raft_small'], 'topic': '/optical_flow/raft_velocity', 'label': 'RAFT (small)'},
        {'bag': bag_paths['raft_large'], 'topic': '/optical_flow/raft_velocity', 'label': 'RAFT (large)'},
        {'bag': bag_paths['LK'], 'topic': '/optical_flow/LK_velocity', 'label': 'LK'}
    ]

    # Extract and normalize ground truth
    gt_times, gt_vels = extract_data(bag_paths['motor'], '/motor/present_velocity', TwistStamped, 'twist.linear.x')
    if len(gt_times) > 0:
        gt_times = gt_times - gt_times[0]  # Normalize to start at 0
        print(f"Ground Truth: {len(gt_times)} messages, time range {min(gt_times)} to {max(gt_times)}, "
              f"velocity range {min(gt_vels)} to {max(gt_vels)}")
    else:
        print("No ground truth data found.")
        return

    # Extract and normalize optical flow data
    methods_data = []
    for method in methods:
        times_of, vels_of = extract_data(method['bag'], method['topic'], Vector3Stamped, 'vector.x')
        if len(times_of) > 0:
            times_of = times_of - times_of[0]  # Normalize to start at 0
            methods_data.append((method, times_of, vels_of))
        else:
            print(f"No data for {method['label']}")
            continue

    # Define common time grid from 0 to maximum end time
    grid_freq = 100.0
    dt = 1.0 / grid_freq
    all_end_times = [gt_times[-1]] + [times_of[-1] for _, times_of, _ in methods_data]
    max_time = max(all_end_times)
    t_grid = np.arange(0, max_time + dt, dt)

    # Interpolate ground truth
    gt_grid = interpolate_to_grid(gt_times, gt_vels, t_grid)
    print(f"Ground Truth interpolated: {np.isnan(gt_grid).sum()} NaNs out of {len(gt_grid)} points")

    # Process optical flow methods and find optimal shifts
    aligned_data = {}
    for method, times_of, vels_of in methods_data:
        try:
            of_grid = interpolate_to_grid(times_of, vels_of, t_grid)
            # Find optimal shift using RMSE
            max_shift_seconds = 1.0
            max_k = int(max_shift_seconds / dt)
            k_range = np.arange(-max_k, max_k + 1)
            errors = [compute_error(gt_grid, of_grid, k) for k in k_range]
            optimal_k = k_range[np.argmin(errors)]
            optimal_shift = optimal_k * dt
            print(f"{method['label']}: optimal shift {optimal_shift:.3f} s, "
                  f"interpolated NaNs: {np.isnan(of_grid).sum()} out of {len(of_grid)} points")
            # Apply optimal shift
            if optimal_k > 0:
                aligned_of = np.concatenate((of_grid[optimal_k:], np.full(optimal_k, np.nan)))
            elif optimal_k < 0:
                aligned_of = np.concatenate((np.full(-optimal_k, np.nan), of_grid[:optimal_k]))
            else:
                aligned_of = of_grid.copy()
            aligned_data[method['label']] = aligned_of
        except ValueError as e:
            print(f"{method['label']} interpolation failed: {e}")

    # Plotting
    plt.figure(figsize=(12, 6))
    plt.plot(t_grid, gt_grid, label='Ground Truth', color='black', linewidth=2)
    colors = ['blue', 'green', 'red', 'purple', 'orange']
    for i, (label, data) in enumerate(aligned_data.items()):
        plt.plot(t_grid, data, label=label, color=colors[i % len(colors)])
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity (m/s)')
    plt.title('Aligned Optical Flow Velocities vs Ground Truth')
    plt.legend()
    plt.grid(True)
    plt.savefig('aligned_velocities.png')
    plt.show()

if __name__ == '__main__':
    main()