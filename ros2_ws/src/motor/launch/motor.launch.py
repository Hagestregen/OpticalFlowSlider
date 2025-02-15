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


    return LaunchDescription([
        motor_node,
    ])
