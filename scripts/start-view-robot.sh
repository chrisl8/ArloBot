#!/usr/bin/env bash

# shellcheck source=/home/chrisl8/ros2_ws/install/setup.bash
source "${HOME}/ros2_ws/install/setup.bash"

echo "set Global Options->Fixed Frame to"
echo "'odom' in order to make this work."
ros2 launch arlobot_ros view_robot.launch
