#!/usr/bin/env bash

# Set up ROS Environment
ROS_HOSTNAME=$(uname -n)
export ROS_HOSTNAME
export ROS_MASTER_URI=http://localhost:11311
export ROSLAUNCH_SSH_UNKNOWN=1
# shellcheck source=/opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
