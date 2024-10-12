#!/usr/bin/env bash
source /opt/ros/jazzy/setup.bash
# https://robotics.stackexchange.com/a/113133
PYTHONWARNINGS="ignore:easy_install command is deprecated,ignore:setup.py install is deprecated"
export PYTHONWARNINGS

ROS2_WS=ros2_ws
cd "${HOME}/${ROS2_WS}" || exit

colcon build --symlink-install
