import rosbag2_py
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import Vector3Stamped, TwistStamped
import numpy as np
from scipy.interpolate import interp1d
from scipy.signal import correlate
import matplotlib.pyplot as plt

# Step 1: Read data from ROS2 bag file
def extract_velocity_data(bag_path, gt_topic, of_topic):
    """
    Extract timestamps and velocities from a ROS2 bag file for given topics.
    
    Args:
        bag_path (str): Path to the ROS2 bag file.
        gt_topic (str): Ground truth topic name.
        of_topic (str): Optical flow topic name.
    
    Returns:
        tuple: (gt_times, gt_velocities), (of_times, of_velocities) as numpy arrays.
    """
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader.open(storage_options, converter_options)

    gt_times, gt_velocities = [], []
    of_times, of_velocities = [], []

    while reader.has_next():
        (topic, data, t) = reader.read_next()
        timestamp = t / 1e9  # Convert nanoseconds to seconds
        if topic == gt_topic:
            msg = deserialize_message(data, TwistStamped)
            velocity = msg.twist.linear.x  # Linear velocity from TwistStamped
            gt_times.append(timestamp)
            gt_velocities.append(velocity)
        elif topic == of_topic:
            msg = deserialize_message(data, Vector3Stamped)
            velocity = msg.vector.x  # X-component velocity from Vector3Stamped
            of_times.append(timestamp)
            of_velocities.append(velocity)

    return (np.array(gt_times), np.array(gt_velocities)), (np.array(of_times), np.array(of_velocities))

# Step 2: Interpolate to a common time grid
def interpolate_to_grid(times, values, t_grid):
    """
    Interpolate velocity data onto a specified time grid.
    
    Args:
        times (np.ndarray): Original timestamps.
        values (np.ndarray): Original velocity values.
        t_grid (np.ndarray): Target time grid for interpolation.
    
    Returns:
        np.ndarray: Interpolated values on t_grid.
    """
    if len(times) < 2:
        raise ValueError("Insufficient data points for interpolation.")
    interp_func = interp1d(times, values, bounds_error=False, fill_value="extrapolate")
    return interp_func(t_grid)

# Step 3 & 4: Find optimal shift using cross-correlation and align
def find_optimal_shift(gt_grid, of_grid, dt):
    """
    Compute the optimal time shift between two signals using cross-correlation.
    
    Args:
        gt_grid (np.ndarray): Ground truth velocity on time grid.
        of_grid (np.ndarray): Optical flow velocity on time grid.
        dt (float): Time step in seconds.
    
    Returns:
        tuple: (shift_steps, tau) where shift_steps is the number of grid steps and tau is the shift in seconds.
    """
    corr = correlate(gt_grid - np.mean(gt_grid), of_grid - np.mean(of_grid), mode='full')
    lags = np.arange(-(len(of_grid) - 1), len(gt_grid))
    tau_idx = np.argmax(corr)
    tau = lags[tau_idx] * dt  # Optimal shift in seconds
    shift_steps = int(round(tau / dt))
    return shift_steps, tau

# Step 5: Compute RMSE
def compute_rmse(gt_data, of_data):
    """
    Compute the Root Mean Square Error between two datasets.
    
    Args:
        gt_data (np.ndarray): Ground truth data.
        of_data (np.ndarray): Optical flow data.
    
    Returns:
        float: RMSE value.
    """
    valid_length = min(len(gt_data), len(of_data))
    return np.sqrt(np.mean((gt_data[:valid_length] - of_data[:valid_length])**2))


# Step 6: Compute MAE
def compute_mae(gt_data, of_data):
    """
    Compute the Mean Absolute Error between two datasets.
    
    Args:
        gt_data (np.ndarray): Ground truth data.
        of_data (np.ndarray): Optical flow data.
    
    Returns:
        float: MAE value.
    """
    valid_length = min(len(gt_data), len(of_data))
    return np.mean(np.abs(gt_data[:valid_length] - of_data[:valid_length]))

# Main execution
def main():
    # Define parameters
    name = 'raft_L_occlusion_960_rmse'
    bag_path = "/home/docker/OpticalFlowSlider/ros2_ws/my_rosbag/Experiment1/Occlusion/raft_L_occlusion_960/raft_L_occlusion_960_0.db3"
    gt_topic = "/motor/present_velocity"
    of_topic = "/optical_flow/raft_velocity"
    grid_freq = 100  # Hz

    # Extract data
    (gt_times, gt_velocities), (of_times, of_velocities) = extract_velocity_data(bag_path, gt_topic, of_topic)

    # Check for sufficient data
    if len(gt_times) < 2 or len(of_times) < 2:
        raise ValueError("Insufficient data points in one of the topics.")

    # Define a common time grid over the overlapping range
    t_start = max(gt_times[0], of_times[0])
    t_end = min(gt_times[-1], of_times[-1])
    if t_start >= t_end:
        raise ValueError("No overlapping time range between ground truth and optical flow data.")
    t_grid = np.arange(t_start, t_end, 1/grid_freq)
    dt = 1 / grid_freq  # Time step in seconds

    # Interpolate both datasets onto the common time grid
    gt_grid = interpolate_to_grid(gt_times, gt_velocities, t_grid)
    of_grid = interpolate_to_grid(of_times, of_velocities, t_grid)

    # Find optimal shift
    shift_steps, tau = find_optimal_shift(gt_grid, of_grid, dt)
    print(f"Optimal shift: {tau:.3f} seconds ({shift_steps} steps)")

    # Align optical flow data
    if shift_steps > 0:
        aligned_of_grid = of_grid[:-shift_steps]
        aligned_gt_grid = gt_grid[shift_steps:]
        plot_t_grid = t_grid[shift_steps:]
    else:
        aligned_of_grid = of_grid[-shift_steps:]
        aligned_gt_grid = gt_grid[:shift_steps]
        plot_t_grid = t_grid[:shift_steps]

    # Compute RMSE
    rmse = compute_rmse(aligned_gt_grid, aligned_of_grid)
    print(f"RMSE after alignment: {rmse:.4f}")
    
    # Compute MAE
    mae = compute_mae(aligned_gt_grid, aligned_of_grid)
    print(f"MAE after alignment: {mae:.4f}")

    # Plot results
    plt.figure(figsize=(12, 6))
    plt.plot(t_grid, gt_grid, label="Ground Truth", color='blue')
    plt.plot(t_grid, of_grid, label="Optical Flow (Original)", color='orange', alpha=0.5)
    plt.plot(plot_t_grid, aligned_of_grid, label="Optical Flow (Aligned)", color='green')
    plt.xlabel("Time (s)")
    plt.ylabel("Velocity (m/s)")
    plt.title("Ground Truth vs Optical Flow Velocity. RMSE: {:.4f}, MAE: {:.4f}".format(rmse, mae))
    plt.legend()
    plt.grid(True)
    plt.savefig(name +".png")
    plt.close()

if __name__ == "__main__":
    main()

