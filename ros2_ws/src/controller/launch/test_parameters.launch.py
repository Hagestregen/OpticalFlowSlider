#!/usr/bin/env python3
import os
import launch
from launch import LaunchDescription
from launch.actions import ExecuteProcess
import launch_ros.actions

def get_unique_bag_folder(base_dir="my_rosbag", base_name="Testfilterparam_2-5_a-0-05_f-0-001_b-0-001"):
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
        executable='kalman_filter_node',
        name='kalman_filter_node',
        parameters=[{
            'dt': 0.01,
            'sigma_a': 0.05,
            'sigma_flow': 0.001,  # Adjusted for LiteFlowNet noise
            'sigma_b': 0.001,
            'imu_cutoff_freq': 2.5,
            'imu_sampling_freq': 71,
            'imu_filter_order': 2,
        }],
        output='screen'
    )

    motor_node = launch_ros.actions.Node(
        package='motor',
        executable='motor_node',
        name='motor_node',
        output='screen'
    )
    
    motor_publisher_node = launch_ros.actions.Node(
        package='motor',
        executable='motor_publisher_node',
        name='motor_publisher_node',
        output='screen'
    )

    data_handler_realsence_accel = launch_ros.actions.Node(
        package='data_handler',
        executable='realsense_accel_node',
        name='realsense_accel_node',
        output='screen'
    )
        
    data_handler_inertialsense_node = launch_ros.actions.Node(
        package='data_handler',
        executable='inertialsense_accel_node',
        name='inertialsense_accel_node',
        output='screen'
    )
        
    optical_flow_node_kanade = launch_ros.actions.Node(
        package='optical_flow',
        executable='lucas_kanade_node',
        name='lucas_kanade_node',
        output='screen'
    )
        
    pid_controller_node = launch_ros.actions.Node(
        package='controller',
        executable='pid_controller_node',
        name='pid_controller_node',
        output='screen'
    )
    
    # Start the RealSense node.
    realsense_node = ExecuteProcess(
        cmd=[
            'ros2', 'launch',
            'realsense2_camera', 'rs_launch.py',

            # Disable depth and IR
            'depth_module.enable:=false',
            'enable_infra1:=false',
            'enable_infra2:=false',

            # Enable color at 640x480
            # 'rgb_camera.color_profile:=640x480x30',     # 640x480x30
            'rgb_camera.color_profile:=1280x720x30',    # 1280x720x30
            # 'rgb_camera.color_profile:=960x540x30',     # 960x540x30
            # 'rgb_camera.color_profile:=848x480x60',     # 848x480x60
            # 'rgb_camera.color_profile:=848x480x30',     # 848x480x30

            # 'enable_color:=true',
            # 'color_width:=640',
            # 'color_height:=480',
            # 'color_fps:=30',

            # Enable IMU (accel)
            'enable_accel:=true'
        ],
        output='screen'
        )




    # Record bag using the unique folder.
    bag_record = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record', '-o', unique_folder,
            # '/camera/camera/color/image_raw',
            # '/optical_flow/LFN_velocity',
            '/optical_flow/LFN_velocity',
            '/events/read_split',
            '/motor/present_velocity',
            '/inertialsense/velocity_x',
            '/kalman_filter/state',
            '/kalman_filter/velocity',
            # '/optical_flow/LK_velocity',
            '/inertialsense/imu',
            '/inertialsense/velocity_no_bias', # Velocity no bias
            '/motor/control_input',
            '/kalman_filter/imu_filtered',
        ],
        output='screen'
    )

    return LaunchDescription([
        # realsense_node,
        # optical_flow_node_kanade,
        bag_record,
        data_handler_inertialsense_node,
        pid_controller_node,
        kalman_filter_cfg_node,
        motor_node,
    ])
    
    

if __name__ == '__main__':
    generate_launch_description()

    