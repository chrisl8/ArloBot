#!/usr/bin/env bash

# shellcheck source=/home/chrisl8/ros2_ws/install/setup.bash
source "${HOME}/ros2_ws/install/setup.bash"

# By default this will use the /cmd_vel topic,
# but we want to use /key_vel instead so that we can use the same topic for both the keyboard and the web interface and the joystick.
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap /cmd_vel:=/key_vel
