#!/usr/bin/env python3
import os
import launch
from launch import LaunchDescription
from launch.actions import ExecuteProcess
import launch_ros.actions
from launch.actions import ExecuteProcess, TimerAction
import launch_ros.actions

def get_unique_bag_folder(base_dir="my_rosbag", base_name="Experiment1_960_occlusion"):
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
    

    motor_node_pos = launch_ros.actions.Node(
        package='motor',
        executable='motor_node',
        name='motor_node',
        output='screen'
    )
    
    depth_calc_node = launch_ros.actions.Node(
        package='data_handler',
        executable='depth_calc_node',
        name='depth_calc_node',
        output='screen'
    )


        
    data_handler_inertialsense_node = launch_ros.actions.Node(
        package='data_handler',
        executable='inertialsense_accel_node',
        name='inertialsense_accel_node',
        output='screen'
    )
    
    # Start the RealSense node.
    realsense_node = ExecuteProcess(
        cmd=[
            'ros2', 'launch',
            'realsense2_camera', 'rs_launch.py',

            # Enable Depth
            'depth_module.enable:=true',
            'depth_module.depth_profile:=640x480x30',
            'align_depth.enable:=true',
            
            
        

            # Enable color at 640x480
            # 'rgb_camera.color_profile:=640x480x30',     # 640x480x30
            # 'rgb_camera.color_profile:=1280x720x30',    # 1280x720x30
            'rgb_camera.color_profile:=960x540x30',     # 960x540x30
            # 'rgb_camera.color_profile:=848x480x60',     # 848x480x60
            # 'rgb_camera.color_profile:=848x480x30',     # 848x480x30

            # 'enable_color:=true',
            # 'color_width:=640',
            # 'color_height:=480',
            # 'color_fps:=30',

            # Enable IMU (accel)
            # 'enable_accel:=true'
        ],
        output='screen'
        )
        
        
    
    
    delayed_motor_play = TimerAction(period=1.5, actions=[motor_node_pos])





    # Record bag using the unique folder.
    bag_record = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record', '-o', unique_folder,
            '/camera/camera/color/image_raw',
            '/events/read_split',
            '/motor/present_velocity',
            '/inertialsense/velocity_x',
            '/inertialsense/imu',
            '/inertialsense/velocity_no_bias', # Velocity no bias
            '/motor/control_input',
            '/slider/current_position',
            '/camera/depth/median_distance',
    
        ],
        output='screen'
    )

    return LaunchDescription([
        bag_record,
        realsense_node,
        depth_calc_node,
        data_handler_inertialsense_node,
        
        # kalman_filter_cfg_node,
        delayed_motor_play
    ])
    
    

if __name__ == '__main__':
    generate_launch_description()

    