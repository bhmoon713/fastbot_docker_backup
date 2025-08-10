#!/bin/bash
set -e
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=/var/lib/theconstruct.rrl/cyclonedds_husarnet.xml

# Process the lsx10.yaml template with environment variable substitution
# This creates a processed copy without modifying the original mounted file
if [ -f "/root/ros2_ws/install/lslidar_driver/share/lslidar_driver/params/lsx10.yaml.template" ]; then
    envsubst < /root/ros2_ws/install/lslidar_driver/share/lslidar_driver/params/lsx10.yaml.template > /root/ros2_ws/install/lslidar_driver/share/lslidar_driver/params/lsx10.yaml
fi

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash" --
source "/root/ros2_ws/install/setup.bash"
exec "$@"
