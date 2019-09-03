#!/usr/bin/env bash

if pgrep -f robot.launch >/dev/null; then
  ROS_HOSTNAME=$(uname -n)
  export ROS_HOSTNAME
  export ROS_MASTER_URI=http://localhost:11311
  export ROSLAUNCH_SSH_UNKNOWN=1
  # shellcheck source=/home/chrisl8/catkin_ws/devel/setup.bash
  source ~/catkin_ws/devel/setup.bash
  rosrun tf tf_echo /map /base_link
else
  echo "Robot must be running to start this."
  exit 1
fi
