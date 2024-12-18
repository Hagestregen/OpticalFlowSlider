FROM docker.mohntechnology.no/companion_base:OF

LABEL description="This is a custom docker image including the ROS2 workspace used by the companion. It is based on the companion_base image that offers a ROS2, Ignition Gazebo and PX4 setup with bridges."

USER docker

# Disable Prompt During Packages Installation
ARG DEBIAN_FRONTEND=noninteractive

# Update and upgrade Ubuntu Software repository
RUN sudo apt-get update
RUN sudo apt-get upgrade -y

# ROS2 workspace
WORKDIR /home/docker
RUN mkdir -p OpticalFlowSlider/ros2_ws/src



### user settings ###
COPY scripts/docker/settings/.vscode /home/docker/OpticalFlowSlider/ros2_ws/.vscode
COPY scripts/docker/settings/.bash_aliases /home/docker/.bash_aliases

# RUN echo "export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=/home/docker/net_inspector/auv_ws/src/auv_simulation/auv_ign/plugin/move_pose/build" >> ~/.bashrc
RUN echo "if [ -e /home/docker/OpticalFlowSlider/ros2_ws/install/setup.bash ]; then . /home/docker/OpticalFlowSlider/ros2_ws/install/setup.bash; fi" >> ~/.bashrc

# get access to video devices and add user to dialout group for serial port access
RUN sudo usermod -a -G video docker
RUN sudo usermod -a -G dialout docker





### Other potentially useful commands ###

# WORKING: Get the ssh key from the host
# git is aleady installed in the base image
# RUN sudo apt-get install -y openssh-client git    
# RUN mkdir -p -m 0700 ~/.ssh 
# COPY --chown=docker:docker scripts/docker/build/.ssh/id_ed25519 /home/docker/.ssh/id_ed25519
# RUN ssh-keyscan -H github.com >> ~/.ssh/known_hosts
# WORKDIR /home/docker
# RUN --mount=type=ssh git clone git@github.com:mohndrilling/auv_ws.git
# RUN rm -r /home/docker/.ssh  # not tested yet
# NB: might have to add --ssh default to the docker build bash script

# Colcon building no packages just to get the setup.bash
# WORKDIR /home/docker/net_inspector/auv_ws
# RUN cbps