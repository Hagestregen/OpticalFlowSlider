#!/usr/bin/env python3
import os
import launch
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.substitutions import LaunchConfiguration
import launch_ros.actions
from launch.actions import TimerAction

def get_unique_bag_folder(base_dir="my_rosbag", base_name="Recording_X"):
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
    # Declare launch argument for base_name with a default value
    base_name_arg = DeclareLaunchArgument(
        'base_name',
        default_value='Recording_X',
        description='Base name for the bag file folder'
    )
    
    # Declare launch argument for existing_bag_path with a default value of an empty string
    existing_bag_path_arg = DeclareLaunchArgument(
        'bag_path',
        default_value='',
        description='Path to the existing ROS 2 bag file to play'
    )
    
    # Log the base name being used
    log_base_name = LogInfo(msg=['Using base name: ', LaunchConfiguration('base_name')])
    
    # Function to generate the bag record process with the resolved base name
    def generate_bag_record(context):
        base_name = LaunchConfiguration('base_name').perform(context)
        unique_folder, bag_name = get_unique_bag_folder(base_name=base_name)
        print(f"Recording to bag folder: {unique_folder}")
        return [ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'record', '-o', unique_folder,
                '/events/read_split',
                # Camera
                '/camera/camera/color/image_raw',
                '/camera/camera/depth/image_raw',
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
                # Kalman Filter
                '/kalman_filter/state',
                '/kalman_filter/velocity',
                '/kalman_filter/imu_filtered',
                '/kalman_filter/new_state',
                '/kalman_filter/new_velocity',
                '/kalman_filter/new_position',
                '/kalman_filter/new_imu_filtered',
                '/kalman_filter/imu_unfiltered',
                'kalman/p_matrix',
                # IMU
                '/inertialsense/imu',
                '/inertialsense/velocity_x',
                '/inertialsense/velocity_no_bias', # Velocity no bias
                # Motor
                '/motor/present_velocity',
                #Pid 
                '/motor/control_input',
                '/slider/current_position',
                '/pid/output',
            ],
            output='screen'
        )]

    # Function to generate the bag play process if existing_bag_path is provided
    def generate_bag_play(context):
        existing_bag_path = LaunchConfiguration('bag_path').perform(context)
        if existing_bag_path:
            print(f"Playing existing bag file: {existing_bag_path}")
            return [ExecuteProcess(
                cmd=['ros2', 'bag', 'play', existing_bag_path],
                output='screen'
            )]
        else:
            print("No existing bag file path provided. Skipping playback.")
            return []

    # #Lucas Kanade
    # kalman_filter_cfg_node = launch_ros.actions.Node(
    #     package='kalman_filter',
    #     executable='LK_kalman_filter_node',
    #     name='LK_kalman_filter_node',
    #     parameters=[{
    #         'dt':                   0.014,
    #         'sigma_a':              0.25,
    #         'sigma_flow':           0.005,  
    #         'sigma_b':              0.05,  
    #         # 'sigma_b':              0.1,   
    #         'imu_cutoff_freq':      20.0,
    #         'imu_sampling_freq':    71,
    #         'imu_filter_order':     2,
    #         'enable_oosm':          True,
    #     }],
    #     output='screen'
    # )
    
    
    # ## LFN3
    # kalman_filter_cfg_node = launch_ros.actions.Node(
    #     package='kalman_filter',
    #     executable='LFN3_kalman_filter_node',
    #     name='LFN3_kalman_filter_node',
    #     parameters=[{
    #         'dt':                   0.014,
    #         'sigma_a':              0.25,
    #         'sigma_flow':           0.05,  
    #         'sigma_b':              0.05,  
    #         # 'sigma_b':              0.1,   
    #         'imu_cutoff_freq':      20.0,
    #         'imu_sampling_freq':    71,
    #         'imu_filter_order':     2,
    #         'enable_oosm':          True,
    #     }],
    #     output='screen'
    # )
    
    ## RAFT
    kalman_filter_cfg_node = launch_ros.actions.Node(
        package='kalman_filter',
        executable='raft_kalman_filter_node',
        name='raft_kalman_filter_node',
        parameters=[{
            'dt':                   0.014,
            'sigma_a':              0.5,
            'sigma_flow':           0.005,  
            'sigma_b':              0.05,  
            # 'sigma_b':              0.1,   
            'imu_cutoff_freq':      15.0,
            'imu_sampling_freq':    71,
            'imu_filter_order':     2,
            'enable_oosm':          True,
        }],
        output='screen'
    )

    return LaunchDescription([
        base_name_arg,
        existing_bag_path_arg,
        log_base_name,
        OpaqueFunction(function=generate_bag_record),
        OpaqueFunction(function=generate_bag_play),
        kalman_filter_cfg_node,
        # delayed_motor_play
    ])

if __name__ == '__main__':
    generate_launch_description()


# import os
# import launch
# from launch import LaunchDescription
# from launch.actions import ExecuteProcess, DeclareLaunchArgument, OpaqueFunction, LogInfo
# from launch.substitutions import LaunchConfiguration
# import launch_ros.actions
# from launch.actions import TimerAction

# def get_unique_bag_folder(base_dir="my_rosbag", base_name="Recording_X"):
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
#     # Declare launch argument for base_name with a default value
#     base_name_arg = DeclareLaunchArgument(
#         'base_name',
#         default_value='Recording_X',
#         description='Base name for the bag file folder'
#     )
    
#     # Log the base name being used
#     log_base_name = LogInfo(msg=['Using base name: ', LaunchConfiguration('base_name')])
    
#     # Function to generate the bag record process with the resolved base name
#     def generate_bag_record(context):
#         base_name = LaunchConfiguration('base_name').perform(context)
#         unique_folder, bag_name = get_unique_bag_folder(base_name=base_name)
#         print(f"Recording to bag folder: {unique_folder}")
#         return [ExecuteProcess(
#             cmd=[
#                 'ros2', 'bag', 'record', '-o', unique_folder,
#                 '/events/read_split',
#                 # Camera
#                 '/camera/camera/color/image_raw',
#                 '/camera/camera/depth/image_raw',
#                 # Depth
#                 '/camera/depth/median_distance',
#                 # Optical flow
#                 '/optical_flow/LFN3_velocity',
#                 '/optical_flow/LFN3_smooth_velocity',
#                 '/optical_flow/PWC_velocity',
#                 '/optical_flow/PWC_smooth_velocity',
#                 '/optical_flow/raft_smooth_velocity',
#                 '/optical_flow/raft_velocity',
#                 '/optical_flow/LK_velocity',
#                 '/optical_flow/LK_smooth_velocity',
#                 '/optical_flow/raw_velocity',
#                 # Kalman Filter
#                 '/kalman_filter/state',
#                 '/kalman_filter/velocity',
#                 '/kalman_filter/imu_filtered',
#                 '/kalman_filter/new_state',
#                 '/kalman_filter/new_velocity',
#                 '/kalman_filter/new_position',
#                 '/kalman_filter/new_imu_filtered',
#                 '/kalman_filter/imu_unfiltered',
#                 'kalman/p_matrix',
#                 # IMU
#                 '/inertialsense/imu',
#                 '/inertialsense/velocity_x',
#                 '/inertialsense/velocity_no_bias', # Velocity no bias
#                 # Motor
#                 '/motor/present_velocity',
#                 #Pid 
#                 '/motor/control_input',
#                 '/slider/current_position',
#                 '/pid/output',
#             ],
#             output='screen'
#         )]

#     # # Define the nodes to be launched
#     # motor_node_pos = launch_ros.actions.Node(
#     #     package='motor',
#     #     executable='motor_kf_node',
#     #     name='motor_kf_node',
#     #     output='screen'
#     # )
    
#     # kalman_filter_cfg_node = launch_ros.actions.Node(
#     #     package='kalman_filter',
#     #     executable='LFN3_kalman_filter_node',
#     #     name='LFN3_kalman_filter_node',
#     #     parameters=[{
#     #         'dt':                   0.014,
#     #         'sigma_a':              0.25,
#     #         'sigma_flow':           0.02,  
#     #         'sigma_b':              0.05,    
#     #         'imu_cutoff_freq':      5.0,
#     #         'imu_sampling_freq':    71,
#     #         'imu_filter_order':     2,
#     #         'enable_oosm':          True,
#     #     }],
#     #     output='screen'
#     # )
    
#     kalman_filter_cfg_node = launch_ros.actions.Node(
#         package='kalman_filter',
#         executable='LK_kalman_filter_node',
#         name='LK_kalman_filter_node',
#         parameters=[{
#             'dt':                   0.014,
#             'sigma_a':              0.25,
#             'sigma_flow':           0.01,  
#             'sigma_b':              0.05,    
#             'imu_cutoff_freq':      5.0,
#             'imu_sampling_freq':    71,
#             'imu_filter_order':     2,
#             'enable_oosm':          True,
#         }],
#         output='screen'
#     )

#     # delayed_motor_play = TimerAction(period=2.5, actions=[motor_node_pos])

#     return LaunchDescription([
#         base_name_arg,
#         log_base_name,
#         OpaqueFunction(function=generate_bag_record),
#         kalman_filter_cfg_node,
#         # delayed_motor_play
#     ])

# if __name__ == '__main__':
#     generate_launch_description()
