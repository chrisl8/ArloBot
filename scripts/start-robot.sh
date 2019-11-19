#!/usr/bin/env bash
# This is the primary script to
# Start the entire robot

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

# shellcheck source=/home/chrisl8/catkin_ws/src/ArloBot/scripts/rosEnvironmentSetup.sh
source "${SCRIPTDIR}/rosEnvironmentSetup.sh"

# shellcheck source=/home/chrisl8/catkin_ws/src/ArloBot/scripts/ros_prep.sh
source "${SCRIPTDIR}/ros_prep.sh"

echo "Use kill_ros.sh to close."

ARLOBOT_MODEL=$(jq '.arlobotModel' "${HOME}/.arlobot/personalDataForBehavior.json" | tr -d '"')
export ARLOBOT_MODEL

# 'unbuffer' is required for running this from the node based 'behavior'
# scripts. Otherwise stdout data is buffered until ROS exits,
# which makes monitoring status impossible.
# http://stackoverflow.com/a/11337310
# http://linux.die.net/man/1/unbuffer
unbuffer roslaunch arlobot_launchers robot.launch --screen

# TODO: Check on what sensor is in use and just start that one here,
#       instead of in the launch file?