#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
import launch_ros.actions

def get_unique_bag_folder(base_dir="my_rosbag", base_name="LFN_record"):
    if not os.path.exists(base_dir):
        os.makedirs(base_dir)
    candidate = os.path.join(base_dir, base_name)
    counter = 0
    while os.path.exists(candidate):
        counter += 1
        candidate = os.path.join(base_dir, f"{base_name}_{counter}")
    return candidate, os.path.basename(candidate)

def generate_launch_description():
    unique_folder, bag_name = get_unique_bag_folder()

    # Launch the optical flow node.
    optical_flow_node = launch_ros.actions.Node(
        package='optical_flow',
        executable='raft_node',
        name='raft_node',
        output='screen'
    )

    # Define bag_play first
    bag_play = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play', '/home/docker/OpticalFlowSlider/ros2_ws/my_rosbag/lucas_kanade/lucas_kanade_0.db3'
        ]
    )

    # Delay bag_play by 3 seconds
    delayed_bag_play = TimerAction(period=5.0, actions=[bag_play])

    # Start recording the specified topics with ros2 bag record
    bag_record = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record', '-o', unique_folder,
            '/camera/camera/color/image_raw',
            '/events/read_split',
            '/realsense_accel_x',
            '/motor/present_velocity',
            '/inertialsense_velocity_x',
            '/realsense_vel_x',
            '/optical_flow/raft_velocity',
            '/optical_flow_velocity',
            
        ],
        output='screen'
    )

    return LaunchDescription([
        bag_play,
        bag_record,
    ])

if __name__ == '__main__':
    generate_launch_description()
