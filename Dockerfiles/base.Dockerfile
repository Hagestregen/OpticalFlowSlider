FROM ubuntu:22.04

LABEL description="This is a custom docker image for a ROS2, Ignition Gazebo, Realsense2 and InertialSense IMU"

# Update package list and install sudo
RUN apt-get update -y && \
    apt-get install -y sudo

# Create a user 'docker' and add it to sudoers
RUN adduser --disabled-password --gecos '' docker && \
    adduser docker sudo && \
    echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
USER docker

SHELL ["/bin/bash", "-c"]
ARG DEBIAN_FRONTEND=noninteractive

WORKDIR /home/docker

# Update Ubuntu repositories and install tzdata
RUN sudo apt-get update -y && \
    sudo apt-get -y install tzdata

# Install basic utilities and development tools
# (Note: g++-8 has been replaced by g++ because Ubuntu 22.04 uses a newer default compiler)
RUN sudo apt-get install --no-install-recommends -y \
    apt-utils \
    lsb-release \
    wget \
    gnupg \
    git \
    python3-vcstools \
    gcc \
    g++ \
    nano

# Install pip and set python3 as default
RUN sudo apt-get install -y \
    python3-pip \
    python-is-python3

RUN pip install --upgrade pip && \
    pip install scipy "numpy<2" pandas matplotlib openpyxl

## (Ignition Edifice installation removed)

## Install ROS2
# Set up locales
RUN sudo apt-get install -y locales && \
    sudo locale-gen en_US en_US.UTF-8 && \
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    export LANG=en_US.UTF-8

# Setup ROS2 repository sources
RUN sudo apt-get install -y software-properties-common && \
    sudo add-apt-repository universe && \
    sudo apt-get update && sudo apt-get install -y curl gnupg2 lsb-release
RUN sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 desktop along with RViz2 and Rqt Graph
RUN sudo apt-get update -y && \
    sudo apt-get upgrade -y && \
    sudo DEBIAN_FRONTEND=noninteractive apt-get install -y \
        ros-humble-desktop \
        ros-humble-rviz2 \
        ros-humble-rqt-graph
RUN source /opt/ros/humble/setup.bash && \
    echo "source /opt/ros/humble/setup.bash" >> /home/docker/.bashrc

# Install colcon build tools
RUN sudo sh -c 'echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
RUN sudo curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
RUN sudo apt-get update && \
    sudo apt-get install -y python3-colcon-common-extensions

# Install rosdep
RUN sudo apt-get update -y && \
    sudo apt-get install -y python3-rosdep && \
    sudo rosdep init && \
    rosdep update

# OpenGL setup
RUN sudo apt-get update -y && \
    sudo apt-get install --no-install-recommends -y \
        dialog \
        mesa-utils \
        binutils \
        x11-apps \
        glmark2 \
        libglu1-mesa-dev \
        freeglut3-dev \
        mesa-common-dev \
        libglew-dev \
        libglfw3-dev \
        libglm-dev

## Install ROS2 ignition gazebo bridge (bridge to Ignition is kept in case you need it later)
RUN export IGNITION_VERSION=edifice && \
    mkdir -p ros_ign_bridge_ws/src && \
    git clone https://github.com/osrf/ros_ign.git -b humble ros_ign_bridge_ws/src
WORKDIR /home/docker/ros_ign_bridge_ws
RUN rosdep install -r --from-paths src -i -y --rosdistro humble && \
    /bin/bash -c "source /opt/ros/humble/setup.bash; colcon build"
RUN source /home/docker/ros_ign_bridge_ws/install/setup.bash && \
    echo "source /home/docker/ros_ign_bridge_ws/install/setup.bash" >> /home/docker/.bashrc
WORKDIR /home/docker/

# --- Install RealSense with ROS2 wrapper ---
# Option 2: Install the ROS-provided RealSense packages.
RUN sudo apt-get update -y && \
    sudo apt-get install -y ros-humble-librealsense2* ros-humble-realsense2-*

### Install additional packages ###
RUN sudo apt-get update -y && \
    sudo apt-get install -y \
        ros-humble-plotjuggler-ros \
        usbutils
