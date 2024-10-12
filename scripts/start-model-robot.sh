#!/usr/bin/env bash

# shellcheck source=/home/chrisl8/ros2_ws/install/setup.bash
source "${HOME}/ros2_ws/install/setup.bash"

# This will launch RVIZ and the ROS Robot Model
# on your local system so that you can view the robot
# model in RVIZ for tweaking the look and part location
# without having to run the entire ROS robot stack.
# I use this to work on the model from my desktop,
# instead of having to do it from the laptop on the robot.
ros2 launch --debug arlobot_ros model_robot.launch.py
