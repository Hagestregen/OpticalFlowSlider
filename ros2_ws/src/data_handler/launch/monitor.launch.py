#!/usr/bin/env python3
import os
import launch
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros.actions

# def get_unique_bag_folder(base_dir="my_rosbag", base_name="pwc_junction_640"):
#     """
#     Generate a unique folder path under base_dir with the base_name.
#     If base_dir/base_name exists, increment a counter until a new folder name is found.
    
#     Returns:
#         candidate (str): The unique folder path (which does not exist yet).
#         bag_name (str): The base name of the folder.
#     """
#     if not os.path.exists(base_dir):
#         os.makedirs(base_dir)
#     candidate = os.path.join(base_dir, base_name)
#     counter = 0
#     while os.path.exists(candidate):
#         counter += 1
#         candidate = os.path.join(base_dir, f"{base_name}_{counter}")
#     return candidate, os.path.basename(candidate)

def generate_launch_description():
    # Create a unique output folder for the bag
    # unique_folder, bag_name = get_unique_bag_folder()
    
    # NODE_NAMES=("lucas_kanade_node" "lucas_kanade_light_node" "lucas_kanade_accurate_node","raft_small_node","raft_direct_node","lfn3_sub_node","pwc_sub_node")


    # Declare the launch argument for the node to monitor
    # node_to_monitor = LaunchConfiguration('node_to_monitor')
    node_to_monitor = 'lucas_kanade_light_node'
    
    declare_node_to_monitor = DeclareLaunchArgument(
        'node_to_monitor',
        default_value='lucas_kanade_node',  # Default node name, override as needed
        description='Name of the node to monitor'
    )

    # # Record bag using the unique folder
    # bag_record = ExecuteProcess(
    #     cmd=[
    #         'ros2', 'bag', 'record', '-o', unique_folder,
    #         '/events/read_split',
    #         # Camera
    #         '/camera/camera/color/image_raw',
    #         '/camera/camera/depth/image_raw',
    #         '/camera/camera/aligned_depth_to_color/image_raw',
    #         # Depth
    #         '/camera/depth/median_distance',
    #         # Optical flow
    #         '/optical_flow/LFN3_velocity',
    #         '/optical_flow/LFN3_smooth_velocity',
    #         '/optical_flow/PWC_velocity',
    #         '/optical_flow/PWC_smooth_velocity',
    #         '/optical_flow/raft_large_smooth_velocity',
    #         '/optical_flow/raft_large_velocity',
    #         '/optical_flow/raft_small_smooth_velocity',
    #         '/optical_flow/raft_small_velocity',
    #         '/optical_flow/LK_velocity',
    #         '/optical_flow/LK_smooth_velocity',
    #         '/optical_flow/raw_velocity',
    #         # IMU
    #         '/inertialsense/imu',
    #         '/inertialsense/velocity_x',
    #         '/inertialsense/velocity_no_bias',
    #         # Motor
    #         '/motor/present_velocity',
    #         # Junction Points
    #         '/junction_detector/junctions',
    #         # ImageVideo
    #         '/optical_flow/image_live_feed',
    #         '/optical_flow/image_flow',
    #         '/optical_flow/image_mask',
    #     ],
    #     output='screen'
    # )

    # Path to your monitoring script (update this to the actual path)
    monitor_script_path = '/home/docker/OpticalFlowSlider/scripts/cpu_gpu/monitor.sh'  # e.g., '/home/user/monitor.sh'

    # Command to run the monitoring script with the node name
    monitor_cmd = [monitor_script_path, node_to_monitor]

    # ExecuteProcess to run the monitoring script
    monitor_process = ExecuteProcess(
        cmd=monitor_cmd,
        output='screen'
    )

    return LaunchDescription([
        declare_node_to_monitor,
        # bag_record,
        monitor_process,
    ])

if __name__ == '__main__':
    generate_launch_description()