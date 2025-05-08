#!/bin/bash
# Script to run optimized Scout Mini simulation

# Source ROS and workspace
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

# Set CPU governor to performance (requires sudo)
echo "Setting CPU to performance mode (may require password)..."
sudo cpupower frequency-set -g performance || echo "Failed to set CPU governor, continuing anyway..."

# Set process priority
echo "Launching optimized Scout Mini with higher process priority..."
nice -n -10 roslaunch scout_mini_optimized scout_mini_optimized.launch
