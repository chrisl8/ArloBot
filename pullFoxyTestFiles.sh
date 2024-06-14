#!/usr/bin/env bash

# TODO: Delete this file when I don't need the notes anymore.

# This script is part of my personal attempt to make Foxy work on this robot,
# but it doesn't work and probably never will.

WSL2_IP=172.22.133.199

# On a remote system with a fresh Ubuntu install run:

# cd;scp 172.22.133.199:/home/chrisl8/catkin_ws/src/ArloBot/pullFoxyTestFiles.sh .;chmod +x pullFoxyTestFiles.sh;./pullFoxyTestFiles.sh;./setup-foxy.sh

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

cd || exit

scp ${WSL2_IP}:/home/chrisl8/catkin_ws/src/ArloBot/setup-foxy.sh .

cd ~/dev_ws/src/ArloBot/arlobot_ros || exit
scp ${WSL2_IP}:"/home/chrisl8/catkin_ws/src/ArloBot/arlobot_ros/srv/*" srv
scp ${WSL2_IP}:"/home/chrisl8/catkin_ws/src/ArloBot/arlobot_ros/msg/*" msg
scp ${WSL2_IP}:/home/chrisl8/catkin_ws/src/ArloBot/arlobot_ros/package-foxy.xml package.xml
scp ${WSL2_IP}:/home/chrisl8/catkin_ws/src/ArloBot/arlobot_ros/CMakeLists-foxy.txt CMakeLists.txt
scp ${WSL2_IP}:/home/chrisl8/catkin_ws/src/ArloBot/arlobot_ros/src/turtlebot_joy-foxy.cpp src/turtlebot_joy.cpp

if ! [[ -d arlobot_ros ]]; then
  mkdir arlobot_ros
fi
touch arlobot_ros/__init__.py

cd ~/dev_ws/src/ArloBot/PropellerCodeForArloBot || exit

touch COLCON_IGNORE

cd || exit

# TODO: See https://docs.ros.org/en/foxy/Contributing/Migration-Guide.html for many helps on migrating existing work to ROS2

# TODO: Follow https://docs.ros.org/en/foxy/Tutorials/Creating-Your-First-ROS2-Package.html
# to discover a proper ROS2 package format.

# This file basically does the minimum to get a working build,
# but in no way assumes a correct or functioning ROS2 package.

# It may even be wise to reimagine this as a "proper" ROS package?
# or to create a new "Robot Anything" actual ROS2 package that is based on this,
# While leaving this to work as is on TwoFlower?
