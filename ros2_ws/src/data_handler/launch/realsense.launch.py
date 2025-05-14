#!/usr/bin/env python3
import os
import launch
from launch import LaunchDescription
from launch.actions import ExecuteProcess
import launch_ros.actions
from launch.actions import ExecuteProcess, TimerAction
import launch_ros.actions

def generate_launch_description():
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
            'rgb_camera.color_profile:=640x480x30',     # 640x480x30
            # 'rgb_camera.color_profile:=1280x720x30',    # 1280x720x30
            # 'rgb_camera.color_profile:=960x540x30',     # 960x540x30
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
    
    return LaunchDescription([
            realsense_node,
            
            # kalman_filter_cfg_node,
        ])
    
    

if __name__ == '__main__':
    generate_launch_description()

    