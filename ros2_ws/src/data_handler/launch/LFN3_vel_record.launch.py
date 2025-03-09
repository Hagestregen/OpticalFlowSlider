#!/usr/bin/env python3
import os
import launch
from launch import LaunchDescription
from launch.actions import ExecuteProcess
import launch_ros.actions
from launch.actions import ExecuteProcess, TimerAction
import launch_ros.actions

def get_unique_bag_folder(base_dir="my_rosbag", base_name="LFN3_vel"):
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
    
    motor_publisher_node = launch_ros.actions.Node(
        package='motor',
        executable='motor_publisher_node',
        name='motor_publisher_node',
        output='screen'
    )

    data_handler_inertialsense_node = launch_ros.actions.Node(
        package='data_handler',
        executable='inertialsense_accel_node',
        name='inertialsense_accel_node',
        output='screen'
    )
        
        
    pid_controller_node = launch_ros.actions.Node(
    package='controller',
    executable='pid_controller_node',
    name='pid_controller_node',
    output='screen',
    parameters=[{
        'Kp': 2.0,
        'Ki': 0.0,
        'Kd': 0.5,
        'setpoint': 0.0,
        'alpha': 0.3,
        'dt': 0.01,
        'integral_limit': 10.0,
        'deadband': 0.01
        }]
    )
    
    delayed_motor_play = TimerAction(period=1.0, actions=[motor_publisher_node])


    # Record bag using the unique folder.
    bag_record = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record', '-o', unique_folder,
            # '/camera/camera/color/image_raw',
            '/optical_flow/LFN3_velocity',
            '/optical_flow/LFN3_smooth_velocity'
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
    
        ],
        output='screen'
    )

    return LaunchDescription([
        bag_record,
        data_handler_inertialsense_node,
        pid_controller_node,
        kalman_filter_cfg_node,
        delayed_motor_play
    ])
    
    

if __name__ == '__main__':
    generate_launch_description()

    