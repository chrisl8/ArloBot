#!/usr/bin/env bash

if [[ $# -ne 1 ]]; then
  echo 'Please provide a map name on the command line.'
else
  "/opt/ros/${ROS_DISTRO}/bin/rosrun" map_server map_saver -f "${HOME}/.arlobot/rosmaps/${1}"
fi
