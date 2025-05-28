#!/usr/bin/env python3
import os
import launch
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, OpaqueFunction, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration
import launch_ros.actions

def get_unique_bag_folder(base_dir="my_rosbag", base_name="Recording_x"):
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

def setup_bag_record(context):
    base_name = LaunchConfiguration('base_name').perform(context)
    unique_folder, bag_name = get_unique_bag_folder(base_name=base_name)
    print(f"Recording to bag folder: {unique_folder}")
    return [ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record', '-o', unique_folder,
            '/camera/camera/color/image_raw',
            '/events/read_split',
            '/motor/present_velocity',
            '/inertialsense/velocity_x',
            '/inertialsense/imu',
            '/inertialsense/velocity_no_bias',
            '/motor/control_input',
            '/slider/current_position',
            '/camera/depth/median_distance',
            '/camera/camera/color/camera_info',
            '/camera/camera/aligned_depth_to_color/image_raw',
            '/camera/camera/depth/image_rect_raw',
            '/camera/camera/depth/camera_info',
            '/camera/camera/aligned_depth_to_color/camera_info',
        ],
        output='screen'
    )]

def generate_launch_description():
    # Declare launch argument for base_name
    base_name_arg = DeclareLaunchArgument(
        'base_name',
        default_value='Recording1_5_960',
        description='Base name for the bag file folder'
    )
    
    # Log the base name being used
    log_base_name = LogInfo(msg=['Using base name: ', LaunchConfiguration('base_name')])
    
    # Define nodes
    motor_node_pos = launch_ros.actions.Node(
        package='motor',
        executable='motor_kf_node',
        name='motor_kf_node',
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
    
    # realsense_node = ExecuteProcess(
    #     cmd=[
    #         'ros2', 'launch',
    #         'realsense2_camera', 'rs_launch.py',
    #         'depth_module.enable:=true',
    #         'depth_module.depth_profile:=640x480x30',
    #         'align_depth.enable:=true',
    #         'rgb_camera.color_profile:=960x540x30',
    #         # 'rgb_camera.color_profile:=640x480x30',     
    #     ],
    #     output='screen'
    # )
    
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
    
    delayed_motor_play = TimerAction(period=1.5, actions=[motor_node_pos])

    return LaunchDescription([
        base_name_arg,
        log_base_name,
        OpaqueFunction(function=setup_bag_record),
        realsense_node,
        depth_calc_node,
        data_handler_inertialsense_node,
        delayed_motor_play
    ])

if __name__ == '__main__':
    generate_launch_description()

# #!/usr/bin/env python3
# import os
# import launch
# from launch import LaunchDescription
# from launch.actions import ExecuteProcess
# import launch_ros.actions
# from launch.actions import ExecuteProcess, TimerAction
# import launch_ros.actions

# def get_unique_bag_folder(base_dir="my_rosbag", base_name="Recording1_5_960"):
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
#     # Do NOT create the folder here; let ros2 bag record create it.
#     return candidate, os.path.basename(candidate)

# def generate_launch_description():
#     # Create a unique output folder for the bag.
#     unique_folder, bag_name = get_unique_bag_folder()
    

#     # motor_node_pos = launch_ros.actions.Node(
#     #     package='motor',
#     #     executable='motor_node',
#     #     name='motor_node',
#     #     output='screen'
#     # )
    
        
#     motor_node_pos = launch_ros.actions.Node(
#         package='motor',
#         executable='motor_kf_node',
#         name='motor_kf_node',
#         output='screen'
#     )
    
#     depth_calc_node = launch_ros.actions.Node(
#         package='data_handler',
#         executable='depth_calc_node',
#         name='depth_calc_node',
#         output='screen'
#     )


        
#     data_handler_inertialsense_node = launch_ros.actions.Node(
#         package='data_handler',
#         executable='inertialsense_accel_node',
#         name='inertialsense_accel_node',
#         output='screen'
#     )
    
#     # Start the RealSense node.
#     realsense_node = ExecuteProcess(
#         cmd=[
#             'ros2', 'launch',
#             'realsense2_camera', 'rs_launch.py',

#             # Enable Depth
#             'depth_module.enable:=true',
#             'depth_module.depth_profile:=640x480x30',
#             'align_depth.enable:=true',
            
            
        

#             # Enable color at 640x480
#             # 'rgb_camera.color_profile:=640x480x30',     # 640x480x30
#             # 'rgb_camera.color_profile:=1280x720x30',    # 1280x720x30
#             'rgb_camera.color_profile:=960x540x30',     # 960x540x30
#             # 'rgb_camera.color_profile:=848x480x60',     # 848x480x60
#             # 'rgb_camera.color_profile:=848x480x30',     # 848x480x30

#             # 'enable_color:=true',
#             # 'color_width:=640',
#             # 'color_height:=480',
#             # 'color_fps:=30',

#             # Enable IMU (accel)
#             # 'enable_accel:=true'
#         ],
#         output='screen'
#         )
        
        
    
    
#     delayed_motor_play = TimerAction(period=1.5, actions=[motor_node_pos])





#     # Record bag using the unique folder.
#     bag_record = ExecuteProcess(
#         cmd=[
#             'ros2', 'bag', 'record', '-o', unique_folder,
#             '/camera/camera/color/image_raw',
#             '/events/read_split',
#             '/motor/present_velocity',
#             '/inertialsense/velocity_x',
#             '/inertialsense/imu',
#             '/inertialsense/velocity_no_bias', # Velocity no bias
#             '/motor/control_input',
#             '/slider/current_position',
#             '/camera/depth/median_distance',
#             '/camera/camera/color/camera_info',
#             '/camera/camera/aligned_depth_to_color/image_raw',
#             '/camera/camera/depth/image_rect_raw',
#             '/camera/camera/depth/camera_info',
#             '/camera/camera/aligned_depth_to_color/camera_info',
            
    
#         ],
#         output='screen'
#     )

#     return LaunchDescription([
#         bag_record,
#         realsense_node,
#         depth_calc_node,
#         data_handler_inertialsense_node,
        
#         # kalman_filter_cfg_node,
#         delayed_motor_play
#     ])
    
    

# if __name__ == '__main__':
#     generate_launch_description()

    