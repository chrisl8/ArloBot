#!/usr/bin/env bash

# Manual test of the XV11 Neato Lidar

# Grab and save the path to this script
# http://stackoverflow.com/a/246128
SOURCE="${BASH_SOURCE[0]}"
while [ -h "$SOURCE" ]; do # resolve $SOURCE until the file is no longer a symlink
  DIR="$(cd -P "$(dirname "$SOURCE")" && pwd)"
  SOURCE="$(readlink "$SOURCE")"
  [[ $SOURCE != /* ]] && SOURCE="$DIR/$SOURCE" # if $SOURCE was a relative symlink, we need to resolve it relative to the path where the symlink file was located
done
SCRIPTDIR="$(cd -P "$(dirname "$SOURCE")" && pwd)"
# echo "${SCRIPTDIR}" # For debugging

"/opt/ros/${ROS_DISTRO}/bin/roscore" &
while ! (rosparam list &>/dev/null); do
  echo "Waiting for roscore to start . . ."
  sleep 1
done
rosrun xv_11_laser_driver neato_laser_publisher _port:="$("${SCRIPTDIR}/find_XVLidar.sh")" _firmware_version:=2 &
roslaunch arlobot_ros view_xv11.launch --screen
"${SCRIPTDIR}/kill_ros.sh"
