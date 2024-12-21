# Stage 1: Base dependencies stage (install dependencies once)
FROM ros:humble-ros-core AS base

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    python3-colcon-common-extensions \
    git \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    libboost-all-dev \
    && rm -rf /var/lib/apt/lists/*

# Stage 2: Clone and build stage
FROM base AS build

# Use /bin/bash for the next RUN command
SHELL ["/bin/bash", "-c"]

# Clone repositories and pull the latest changes every time
WORKDIR /root/ros2_ws/src
RUN git clone https://bitbucket.org/theconstructcore/fastbot.git \
    && git clone -b ros2 https://github.com/allenh1/rplidar_ros.git \
    && git clone https://github.com/AlexKaravaev/ros2_laser_scan_matcher.git \
    && git clone https://github.com/AlexKaravaev/csm.git

# Build the workspace
WORKDIR /root/ros2_ws
RUN rm -rf src/fastbot/docker/volumes/ros2_laser_scan_matcher && source /ros_entrypoint.sh && colcon build && rm -rf /root/ros2_ws/src

# Stage 3: Final runtime stage
FROM ros:humble-ros-core

RUN apt-get update && apt-get install -y \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-v4l2-camera \
    ros-humble-image-transport-plugins \
    ros-humble-control-msgs \
    python3-pip \
    i2c-tools \
    libgpiod2 \
    && rm -rf /var/lib/apt/lists/* \
    && pip install RPi.GPIO pyserial adafruit-circuitpython-ina219 smbus2   

# Copy built workspace from the build stage
COPY --from=build /root/ros2_ws /root/ros2_ws

ENTRYPOINT ["/ros_entrypoint.sh"]
