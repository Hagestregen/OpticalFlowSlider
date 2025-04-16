FROM docker.mohntechnology.no/companion_base:OF

LABEL description="This is a custom docker image including the ROS2 workspace used by the companion. It is based on the companion_base image that offers a ROS2, Ignition Gazebo and PX4 setup with bridges."

USER docker

# Disable Prompt During Packages Installation
ARG DEBIAN_FRONTEND=noninteractive

# Define ROS distribution
ENV ROS_DISTRO=humble 

# Update and upgrade Ubuntu Software repository
RUN sudo apt-get update --allow-releaseinfo-change
RUN sudo apt-get upgrade -y --fix-missing

# ROS2 workspace
WORKDIR /home/docker
# RUN mkdir -p OpticalFlowSlider/ros2_ws/src
# RUN mkdir -p OpticalFlowSlider/ros_ws/src

### user settings ###
COPY scripts/docker/settings/.vscode /home/docker/OpticalFlowSlider/ros2_ws/.vscode
COPY scripts/docker/settings/.bash_aliases /home/docker/.bash_aliases

RUN echo "if [ -e /home/docker/OpticalFlowSlider/ros2_ws/install/setup.bash ]; then . /home/docker/OpticalFlowSlider/ros2_ws/install/setup.bash; fi" >> ~/.bashrc

# get access to video devices and add user to dialout group for serial port access
RUN sudo usermod -a -G video docker
RUN sudo usermod -a -G dialout docker

# Install dependencies for pyrealsense2
RUN sudo apt-get install -y libssl-dev libusb-1.0-0-dev libudev-dev pkg-config libgtk-3-dev

# Install pyrealsense2
RUN pip install pyrealsense2

# Install dependencies for Dynamixel SDK
RUN sudo apt-get install -y git python3-colcon-common-extensions

# Install PyTorch 1.13.1 with CUDA 11.7 support
RUN pip install torch==1.13.1+cu117 -f https://download.pytorch.org/whl/torch_stable.html

# The remaining (commented out) Dynamixel SDK build steps remain unchanged.
