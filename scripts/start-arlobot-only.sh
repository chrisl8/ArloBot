#!/usr/bin/env bash
# Start just the basic ArloBot ROS setup.
# Use start-robot.sh to start EVERYTHING instead

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
roslaunch arlobot_bringup minimal.launch --screen
