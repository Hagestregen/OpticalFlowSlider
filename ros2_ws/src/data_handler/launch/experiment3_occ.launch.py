#!/usr/bin/env python3
import os
import launch
from launch import LaunchDescription
from launch.actions import ExecuteProcess
import launch_ros.actions
from launch.actions import ExecuteProcess, TimerAction
import launch_ros.actions

def get_unique_bag_folder(base_dir="my_rosbag", base_name="lfn3_NoJunction_three_fish_small_median_640"):
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
    
    
    optical_flow_node_kanade = launch_ros.actions.Node(
        package='optical_flow',
        executable='lucas_kanade_node',
        name='lucas_kanade_node',
        output='screen'
    )
    
    raft_node = launch_ros.actions.Node(
        package='optical_flow',
        executable='raft_direct_node',
        name='raft_direct_node',
        output='screen'
    )



    # Record bag using the unique folder.
    bag_record = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record', '-o', unique_folder,
            '/events/read_split',
            # Camera
            '/camera/camera/color/image_raw',
            '/camera/camera/depth/image_raw',
            '/camera/camera/aligned_depth_to_color/image_raw',
            # Depth
            '/camera/depth/median_distance',
            # Optical flow
            '/optical_flow/LFN3_velocity',
            '/optical_flow/LFN3_smooth_velocity',
            '/optical_flow/PWC_velocity',
            '/optical_flow/PWC_smooth_velocity',
            '/optical_flow/raft_large_smooth_velocity',
            '/optical_flow/raft_large_velocity',
            '/optical_flow/raft_small_smooth_velocity',
            '/optical_flow/raft_small_velocity',
            '/optical_flow/LK_velocity',
            '/optical_flow/LK_smooth_velocity',
            '/optical_flow/raw_velocity',
            # # Kalman Filter
            # '/kalman_filter/state',
            # '/kalman_filter/velocity',
            # '/kalman_filter/imu_filtered',
            # '/kalman_filter/new_state',
            # '/kalman_filter/new_velocity',
            # '/kalman_filter/new_position',
            # '/kalman_filter/new_imu_filtered',
            # IMU
            '/inertialsense/imu',
            '/inertialsense/velocity_x',
            '/inertialsense/velocity_no_bias', # Velocity no bias
            # Motor
            '/motor/present_velocity',
            # #Pid 
            # '/motor/control_input',
            # '/slider/current_position',
            # '/pid/output',
            # Junction Points
            '/junction_detector/junctions',
            # ImageVideo
            '/optical_flow/image_live_feed',
            '/optical_flow/image_flow',
            '/optical_flow/image_mask',
            
    
        ],
        output='screen'
    )

    return LaunchDescription([
        bag_record,
        # optical_flow_node_kanade,
        # raft_node,
        
    ])
    
    

if __name__ == '__main__':
    generate_launch_description()

    