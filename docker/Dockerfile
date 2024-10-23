# Stage 1: Build stage
FROM ros:humble-ros-core AS build

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    python3-colcon-common-extensions \
    git \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    libboost-all-dev \
    && rm -rf /var/lib/apt/lists/*

# Use /bin/bash for the next RUN command
SHELL ["/bin/bash", "-c"]

RUN source /ros_entrypoint.sh \
    && mkdir -p /root/ros2_ws/src \
    && cd /root/ros2_ws/src \
    && git clone https://bitbucket.org/theconstructcore/fastbot.git \
    && git clone -b ros2 https://github.com/allenh1/rplidar_ros.git \
    && git clone https://github.com/AlexKaravaev/ros2_laser_scan_matcher.git \
    && git clone https://github.com/AlexKaravaev/csm.git \
    && cd /root/ros2_ws \
    && colcon build \
    && rm -rf /root/ros2_ws/src \
    && source install/setup.bash

FROM ros:humble-ros-core

RUN apt-get update && apt-get install -y \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-v4l2-camera \
    ros-humble-image-transport-plugins \
    ros-humble-control-msgs \
    python3-pip \
    && rm -rf /var/lib/apt/lists/* \
    && pip install RPi.GPIO 

# Copy built workspace from the build stage
COPY --from=build /root/ros2_ws /root/ros2_ws

ENTRYPOINT ["/ros_entrypoint.sh"]

