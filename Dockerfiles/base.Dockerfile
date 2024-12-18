FROM ubuntu:20.04

LABEL description="This is a custom docker image for a ROS2, Ignition Gazebo, Realsense2 and InertialSense IMU"

RUN apt-get update -y
RUN apt-get install -y sudo

RUN adduser --disabled-password --gecos '' docker
RUN adduser docker sudo
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
USER docker

SHELL ["/bin/bash", "-c"]
ARG DEBIAN_FRONTEND=noninteractive

WORKDIR /home/docker

# Update Ubuntu Software repository
RUN sudo apt-get update -y
RUN sudo apt-get -y install tzdata

# install utilities
# if docker stalls here because you can't select a timezone try to do it from within the intermediate container
RUN sudo apt-get install --no-install-recommends -y \
    apt-utils \
    lsb-release \
    wget \
    gnupg \
    git \
    python3-vcstools \
    gcc \
    g++-8 \
    nano

# install pip and make python3 the default
RUN sudo apt-get install -y \
    python3-pip \
    python-is-python3

RUN pip install --upgrade pip
RUN pip install scipy numpy pandas matplotlib openpyxl

## install ignition gazebo from binaries
RUN sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
RUN sudo wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
RUN sudo apt-get update
RUN sudo apt-get install -y ignition-edifice

## install ROS2
# set locales
RUN sudo apt-get install -y locales
RUN sudo locale-gen en_US en_US.UTF-8
RUN sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
RUN export LANG=en_US.UTF-8

# setup sources
RUN sudo apt-get install -y software-properties-common
RUN sudo add-apt-repository universe
RUN sudo apt-get update && sudo apt-get install -y curl gnupg2 lsb-release
RUN sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# install ROS2 packages
RUN sudo apt-get update -y
RUN sudo apt-get upgrade -y
# if docker stalls here because you can't select a keyboard layout try running the rest from within the intermediate container
RUN sudo DEBIAN_FRONTEND=noninteractive apt-get install -y ros-foxy-desktop
RUN source /opt/ros/foxy/setup.bash
RUN echo "source /opt/ros/foxy/setup.bash" >> /home/docker/.bashrc

# install colcon
RUN sudo sh -c 'echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
RUN sudo curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
RUN sudo apt-get update
RUN sudo apt-get install -y python3-colcon-common-extensions

# install rosdep
RUN sudo apt-get update -y
RUN sudo apt-get install -y python3-rosdep
RUN sudo rosdep init
RUN rosdep update

# openGL setup
RUN sudo apt-get update -y
RUN sudo apt-get install --no-install-recommends -y \
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



## install ROS2 ignition gazebo bridge
RUN export IGNITION_VERSION=edifice
RUN mkdir -p ros_ign_bridge_ws/src
RUN git clone https://github.com/osrf/ros_ign.git -b foxy ros_ign_bridge_ws/src
WORKDIR /home/docker/ros_ign_bridge_ws
RUN rosdep install -r --from-paths src -i -y --rosdistro foxy
RUN /bin/bash -c "source /opt/ros/foxy/setup.bash; colcon build"
RUN source /home/docker/ros_ign_bridge_ws/install/setup.bash
RUN echo "source /home/docker/ros_ign_bridge_ws/install/setup.bash" >> /home/docker/.bashrc
WORKDIR /home/docker/


# # install missing python modules
# RUN pip3 install kconfiglib jinja2 jsonschema toml future


# install RealSense with ROS2 wrapper
RUN sudo mkdir -p /etc/apt/keyrings
RUN curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null
RUN sudo apt-get install apt-transport-https -y
RUN echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
sudo tee /etc/apt/sources.list.d/librealsense.list
RUN sudo apt-get update
RUN sudo apt-get install librealsense2-dkms -y
RUN sudo apt-get install librealsense2-utils -y
RUN sudo apt-get update 
RUN sudo apt-get upgrade -y
RUN sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
RUN sudo apt install ros-foxy-realsense2-* -y



### Install InertialSense cltool ###
# Set the working directory
WORKDIR /home/docker

# Create a directory and clone the repository
RUN git clone --recurse-submodules https://github.com/inertialsense/inertial-sense-sdk.git

# Change the working directory
WORKDIR /home/docker/inertial-sense-sdk/cltool

# Create a build directory and run cmake and make
RUN mkdir build && \
    cd build && \
    cmake .. && \
    make



### CAN bus with dronecan and dronecan_gui_tool
RUN sudo apt-get install -y python3-setuptools python3-wheel
RUN sudo apt-get install -y python3-numpy python3-pyqt5 python3-pyqt5.qtsvg git-core
RUN pip install dronecan pyserial
RUN sudo pip3 install git+https://github.com/DroneCAN/gui_tool@master

# mraa install
RUN sudo apt-get install software-properties-common -y
RUN sudo add-apt-repository ppa:mraa/mraa -y
RUN sudo apt-get update -y
RUN sudo apt-get install --no-install-recommends -y \
    mraa-tools \
    mraa-examples \
    libmraa2 \
    libmraa-dev \
    libupm-dev \
    libupm2 \
    upm-examples \
    python-mraa \
    libmraa-java \
    python3-mraa 

###  install additional packages ###
RUN sudo apt-get update -y
RUN sudo apt-get install -y \
    ros-foxy-plotjuggler-ros \
    ros-foxy-casadi-vendor \
    usbutils

RUN pip install ultralytics