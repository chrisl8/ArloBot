#!/bin/bash

set -e

ECHO_PREFIX="[docker-entrypoint.sh]"

export ROSLAUNCH_SSH_UNKNOWN=1

echo "${ECHO_PREFIX}" "set ROS master: " "${ROS_MASTER_URI}"

# ROS installation
ROS=/opt/ros/noetic/setup.bash
# shellcheck source=/opt/ros/noetic/setup.bash
source "${ROS}"
echo "${ECHO_PREFIX}" "sourced ROS installation:" "${ROS}"

# workspace holding custom ROS packages
workspace=/dev_ws
# shellcheck source=/home/chrisl8/dev_ws/install/setup.bash
source "${workspace}"/install/setup.bash
echo "${ECHO_PREFIX}" "sourced workspace:" "${workspace}"

echo "${ECHO_PREFIX}" "call" "$@"
exec "$@"
