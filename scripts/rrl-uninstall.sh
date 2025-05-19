#!/bin/bash

echo "Uninstalling RRL..."
rrl-uninstall

echo "Removing RRL directory..."
sudo rm -rf /var/lib/theconstruct.rrl/

echo "Removing environment variables from .bashrc..."
sed -i '/^export RMW_IMPLEMENTATION=/d' ~/.bashrc
sed -i '/^export ROS_IPV6=/d' ~/.bashrc
sed -i '/^export ROS_MASTER_URI=/d' ~/.bashrc
sed -i '/^export ROS_HOSTNAME=/d' ~/.bashrc
sed -i '/^export CYCLONEDDS_URI=/d' ~/.bashrc
sed -i '/^export FASTRTPS_DEFAULT_PROFILES_FILE=/d' ~/.bashrc

echo "Sourcing .bashrc to apply changes..."
source ~/.bashrc

echo "Uninstallation completed."
