#!/usr/bin/env bash

set -e

source /opt/ros/jazzy/setup.bash

# https://robotics.stackexchange.com/a/113133
PYTHONWARNINGS="ignore:easy_install command is deprecated,ignore:setup.py install is deprecated"
export PYTHONWARNINGS

ROS2_WS=ros2_ws

if ! [[ -d ${HOME}/${ROS2_WS}/src ]]; then
  printf "\n${YELLOW}[Creating the ROS Development workspace and testing with colcon]${NC}\n"
  mkdir -p "${HOME}/${ROS2_WS}/src"
  cd "${HOME}/${ROS2_WS}"
  colcon build --symlink-install
fi

# shellcheck source=/home/chrisl8/${ROS2_WS}/install/setup.bash
source "${HOME}/${ROS2_WS}/install/setup.bash"

cd "${HOME}/${ROS2_WS}"

colcon build --symlink-install
