#!/usr/bin/env bash

# shellcheck source=/home/chrisl8/ros2_ws/install/setup.bash
source "${HOME}/ros2_ws/install/setup.bash"

ros2 launch arlobot_ros keyboard_teleop.launch.py
