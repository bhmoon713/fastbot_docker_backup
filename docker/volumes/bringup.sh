#!/bin/bash

ros2 launch fastbot_bringup bringup.launch.xml robot_name:=fastbot_1 &
sleep 5
ros2 launch fastbot_bringup rplidar.launch.xml robot_name:=fastbot_1
