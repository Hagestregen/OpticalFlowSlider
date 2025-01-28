if [ -e ~/git_repos/OpticalFlowSlider/ros2_ws ]; then
    cd "${HOME}/git_repos/OpticalFlowSlider/ros2_ws" #Not docker path
else
    cd "${HOME}/OpticalFlowSlider/ros2_ws" #Docker path
fi
# colcon build --packages-select \
#     common_functions_library \
#     inertialsense_msgs

# . install/setup.bash


colcon build --packages-select \
    controller_node \
    data_handler_node \
    motor_node \
    optical_flow_node 

. install/setup.bash

printf "\nROS2 full workspace ready...\n\n"