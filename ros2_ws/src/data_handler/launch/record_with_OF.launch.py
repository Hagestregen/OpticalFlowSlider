#!/usr/bin/env python3
import os
import launch
from launch import LaunchDescription
from launch.actions import ExecuteProcess
import launch_ros.actions

def get_unique_bag_folder(base_dir="my_rosbag", base_name="KalmanFilter_LK_sigma_b_0-1_sigma_a_0-1_with_oosm"):
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

    # Launch the motor node.
    motor_node = launch_ros.actions.Node(
        package='motor',
        executable='motor_node',
        name='motor_node',
        output='screen'
    )

    # Launch data handler nodes.
    data_handler_node = launch_ros.actions.Node(
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

    # Launch the optical flow node.
    optical_flow_node_kanade = launch_ros.actions.Node(
        package='optical_flow',
        executable='lucas_kanade_node',
        name='lucas_kanade_node',
        output='screen'
    )
    
    optical_flow_node_raft = launch_ros.actions.Node(
        package='optical_flow',
        executable='raft_direct_node',
        name='raft_direct_node',
        output='screen'
    )
    
    kalman_filter_node = launch_ros.actions.Node(
        package='kalman_filter',
        executable='kalman_filter_node',
        name='kalman_filter_node',
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
        # 'rgb_camera.color_profile:=640x480x30',

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
            # '/optical_flow_velocity',
            # 'optical_flow/raft_velocity',
            # '/optical_flow/LFN_velocity',
            # '/camera/camera/color/image_raw',
            '/events/read_split',
            '/realsense_accel_x',
            '/motor/present_velocity',
            '/inertialsense_velocity_x',
            '/realsense_vel_x',
            '/kalman_filter/state',
            '/kalman_filter/velocity',
            '/optical_flow/LK_velocity',
        ],
        output='screen'
    )

    return LaunchDescription([
        realsense_node,
        # optical_flow_node_raft,
        optical_flow_node_kanade,
        data_handler_node,
        data_handler_inertialsense_node,
        kalman_filter_node,
        bag_record,
        motor_node,
    ])

if __name__ == '__main__':
    generate_launch_description()
