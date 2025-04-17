#!/usr/bin/env python3
import os
import launch
from launch import LaunchDescription
from launch.actions import ExecuteProcess
import launch_ros.actions
from launch.actions import ExecuteProcess, TimerAction
import launch_ros.actions

def get_unique_bag_folder(base_dir="my_rosbag", base_name="Experiment1_test_flow"):
    """
    Generate a unique folder path under base_dir with the base_name.
    If base_dir/base_name exists, increment a counter until a new folder name is found.
    
    Returns:
        candidate (str): The unique folder path (which does not exist yet).
        bag_name (str): The base name of the folder.
    """
    if not os.path.exists(base_dir):
        os.makedirs(base_dir)
    candidate = os.path.join(base_dir, base_name)
    counter = 0
    while os.path.exists(candidate):
        counter += 1
        candidate = os.path.join(base_dir, f"{base_name}_{counter}")
    # Do NOT create the folder here; let ros2 bag record create it.
    return candidate, os.path.basename(candidate)

def generate_launch_description():
    # Create a unique output folder for the bag.
    unique_folder, bag_name = get_unique_bag_folder()
    
    kalman_filter_cfg_node = launch_ros.actions.Node(
        package='kalman_filter',
        executable='LFN3_kalman_filter_node',
        name='LFN3_kalman_filter_node',
        parameters=[{
            'dt': 0.01,
            'sigma_a': 0.3,
            'sigma_flow': 0.01,  # Adjusted for LiteFlowNet noise
            'sigma_b': 0.005,
            'imu_cutoff_freq': 2.5,
            'imu_sampling_freq': 71,
            'imu_filter_order': 2,
        }],
        output='screen'
    )



    # Record bag using the unique folder.
    bag_record = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record', '-o', unique_folder,
            '/camera/camera/color/image_raw',
            '/optical_flow/LFN3_velocity',
            '/optical_flow/LFN3_smooth_velocity',
            '/events/read_split',
            '/motor/present_velocity',
            '/inertialsense/velocity_x',
            '/kalman_filter/state',
            '/kalman_filter/velocity',
            '/inertialsense/imu',
            '/inertialsense/velocity_no_bias', # Velocity no bias
            '/motor/control_input',
            '/kalman_filter/imu_filtered',
            '/slider/current_position',
            '/pid/output',
            '/camera/depth/median_distance',
    
        ],
        output='screen'
    )

    return LaunchDescription([
        bag_record,
        kalman_filter_cfg_node,
        
        
    ])
    
    

if __name__ == '__main__':
    generate_launch_description()

    