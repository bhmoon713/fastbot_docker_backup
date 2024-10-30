#!/bin/bash

ros2 launch fastbot_bringup bringup.launch.xml
sleep 5
ros2 launch fastbot_bringup rplidar.launch.xml
