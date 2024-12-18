alias cbps="colcon build --packages-select"
alias update_pid="ros2 param load /pid_controller ~/OpticalFlowSlider/ros2_ws/src/auv_common/gnc/controll/pid_controller/config/pid_config.yaml"
alias update_sliding="ros2 param load /sliding_controller ~/OpticalFlowSlider/ros2_ws/src/auv_common/gnc/controll/sliding_controller/config/sliding_config.yaml"
alias colcon_clean="rm -rf build/ install/ log/"
alias config="--ros-args --params-file"
alias arming='ros2 service call /armed_toggle auv_interfaces/ArmedToggle "{data: true}" '
alias disarming='ros2 service call /armed_toggle auv_interfaces/ArmedToggle "{data: false}"'