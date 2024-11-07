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

# Clone repositories and pull the latest changes every time
WORKDIR /root/ros2_ws/src
RUN git clone https://bitbucket.org/theconstructcore/fastbot.git || true \
    && cd fastbot && git pull origin master && cd .. \
    && git clone -b ros2 https://github.com/allenh1/rplidar_ros.git || true \
    && cd rplidar_ros && git pull origin ros2 && cd .. \
    && git clone https://github.com/AlexKaravaev/ros2_laser_scan_matcher.git || true \
    && cd ros2_laser_scan_matcher && git pull origin main && cd .. \
    && git clone https://github.com/AlexKaravaev/csm.git || true \
    && cd csm && git pull origin ros2_csm_eigen && cd ..

# Build the workspace
WORKDIR /root/ros2_ws
RUN source /ros_entrypoint.sh \
    && colcon build \
    && rm -rf /root/ros2_ws/src \
    && source install/setup.bash

# Stage 2: Run stage
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
    && pip install RPi.GPIO pyserial

# Copy built workspace from the build stage
COPY --from=build /root/ros2_ws /root/ros2_ws

ENTRYPOINT ["/ros_entrypoint.sh"]
