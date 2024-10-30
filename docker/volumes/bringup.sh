#!/bin/bash

# Use the ROBOT_NAME environment variable, or default to fastbot_1 if unset
robot_name=${ROBOT_NAME:-fastbot_1}

ros2 launch fastbot_bringup bringup.launch.xml robot_name:=$robot_name &
sleep 5
ros2 launch fastbot_bringup rplidar.launch.xml robot_name:=$robot_name
