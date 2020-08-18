#!/usr/bin/env bash

if [[ $# -ne 1 ]]; then
  echo 'Please provide a map name on the command line.'
else
  "/opt/ros/${ROS_DISTRO}/bin/rosservice" call /slam_toolbox/serialize_map "${HOME}/.arlobot/rosmaps/${1}"
fi
