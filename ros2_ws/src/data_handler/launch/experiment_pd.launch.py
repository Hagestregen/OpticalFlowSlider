#!/usr/bin/env python3
import os
import launch
from launch import LaunchDescription
from launch.actions import ExecuteProcess
import launch_ros.actions
from launch.actions import ExecuteProcess, TimerAction
import launch_ros.actions

def get_unique_bag_folder(base_dir="my_rosbag", base_name="LFN3_PD_Motor"):
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
    

    # motor_node_pos = launch_ros.actions.Node(
    #     package='motor',
    #     executable='motor_node',
    #     name='motor_node',
    #     output='screen'
    # )
    
    
    motor_publisher_node = launch_ros.actions.Node(
        package='motor',
        executable='motor_publisher_node',
        name='motor_publisher_node',
        output='screen'
    )
    
    motor_node_pos = launch_ros.actions.Node(
        package='motor',
        executable='motor_kf_node',
        name='motor_kf_node',
        output='screen'
    )
    # depth_calc_node = launch_ros.actions.Node(
    #     package='data_handler',
    #     executable='depth_calc_node',
    #     name='depth_calc_node',
    #     output='screen'
    # )


        
    data_handler_inertialsense_node = launch_ros.actions.Node(
        package='data_handler',
        executable='inertialsense_accel_node',
        name='inertialsense_accel_node',
        output='screen'
    )
    
    kalman_filter_cfg_node = launch_ros.actions.Node(
        package='kalman_filter',
        executable='LFN3_kalman_filter_node',
        name='LFN3_kalman_filter_node',
        parameters=[{
        'dt':                   0.014,
        'sigma_a':              0.25,
        'sigma_flow':           0.02,  
        'sigma_b':              0.05,    
        'imu_cutoff_freq':      5.0,
        'imu_sampling_freq':    71,
        'imu_filter_order':     2,
        'enable_oosm':          True,
        }],
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
        'setpoint': -0.1,
        'alpha': 0.3,
        'dt': 0.01,
        'integral_limit': 10.0,
        'deadband': 0.01
        }]
    )

        
        
    
    
    # delayed_motor_play = TimerAction(period=2.5, actions=[motor_node_pos])





    # Record bag using the unique folder.
    bag_record = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record', '-o', unique_folder,
            '/events/read_split',
            # Camera
            '/camera/camera/color/image_raw',
            '/camera/camera/depth/image_raw',
            # Depth
            '/camera/depth/median_distance',
            # Optical flow
            '/optical_flow/LFN3_velocity',
            '/optical_flow/LFN3_smooth_velocity',
            '/optical_flow/PWC_velocity',
            '/optical_flow/PWC_smooth_velocity',
            '/optical_flow/raft_smooth_velocity',
            '/optical_flow/raft_velocity',
            '/optical_flow/LK_velocity',
            '/optical_flow/LK_smooth_velocity',
            '/optical_flow/raw_velocity',
            # Kalman Filter
            '/kalman_filter/state',
            '/kalman_filter/velocity',
            '/kalman_filter/imu_filtered',
            '/kalman_filter/new_state',
            '/kalman_filter/new_velocity',
            '/kalman_filter/new_position',
            '/kalman_filter/new_imu_filtered',
            '/kalman_filter/imu_unfiltered',
            'kalman/p_matrix',
            # IMU
            '/inertialsense/imu',
            '/inertialsense/velocity_x',
            '/inertialsense/velocity_no_bias', # Velocity no bias
            # Motor
            '/motor/present_velocity',
            #Pid 
            '/motor/control_input',
            '/slider/current_position',
            '/pid/output',
            
    
        ],
        output='screen'
    )


    return LaunchDescription([
        bag_record,
        # realsense_node,
        # depth_calc_node,
        pid_controller_node,
        data_handler_inertialsense_node,
        kalman_filter_cfg_node,
        motor_publisher_node,
        # motor_node_pos,
    ])
    
    

if __name__ == '__main__':
    generate_launch_description()

    