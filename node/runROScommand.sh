#!/bin/bash
export ROS_HOSTNAME=`uname -n`.local
export ROS_MASTER_URI=http://localhost:11311
export ROSLAUNCH_SSH_UNKNOWN=1
source ~/catkin_ws/devel/setup.bash
${1}
