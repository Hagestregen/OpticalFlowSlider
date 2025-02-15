#!/usr/bin/env python3
import launch
from launch import LaunchDescription
from launch.actions import ExecuteProcess
import launch_ros.actions

def generate_launch_description():
    # Launch the motor node
    motor_node = launch_ros.actions.Node(
        package='motor',            # your package name
        executable='motor_node',         # your executable name (installed entry point)
        name='motor_node',
        output='screen'
    )

    data_handler_node = launch_ros.actions.Node(
        package='data_handler',
        executable='realsense_accel_node',
        name='realsense_accel_node',
        output='screen'
    )

    # Start recording the specified topics with ros2 bag record
    bag_record = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record', '-o', 'my_rosbag',
            '/camera/camera/color/image_raw',
            '/events/read_split',
            '/linear_accel_x',
            '/motor/present_velocity',
            # '/parameter_events',
            # '/pimu_dvel_x',
            # '/rosout'
        ],
        output='screen'
    )

    return LaunchDescription([
        motor_node,
        data_handler_node,
        bag_record,
        
    ])
