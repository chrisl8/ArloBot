#!/bin/bash
# This is the primary script to
# Start the entire robot

# Set up ROS Environment
export ROS_HOSTNAME=`uname -n`
export ROS_MASTER_URI=http://localhost:11311
export ROSLAUNCH_SSH_UNKNOWN=1
source ~/catkin_ws/devel/setup.bash

# Grab and save the path to this script
# http://stackoverflow.com/a/246128
SOURCE="${BASH_SOURCE[0]}"
while [ -h "$SOURCE" ]; do # resolve $SOURCE until the file is no longer a symlink
  DIR="$( cd -P "$( dirname "$SOURCE" )" && pwd )"
  SOURCE="$(readlink "$SOURCE")"
  [[ $SOURCE != /* ]] && SOURCE="$DIR/$SOURCE" # if $SOURCE was a relative symlink, we need to resolve it relative to the path where the symlink file was located
done
SCRIPTDIR="$( cd -P "$( dirname "$SOURCE" )" && pwd )"
# echo ${SCRIPTDIR} # For debugging

source ${SCRIPTDIR}/ros_prep.sh

echo "Use kill_ros.sh to close."

export ARLOBOT_MODEL=$(jq '.arlobotModel' ${HOME}/.arlobot/personalDataForBehavior.json | tr -d '"')

# 'unbuffer' is required for running this from the node based 'behavior'
# scripts. Otherwise stdout data is buffered until ROS exits,
# which makes monitoring status impossible.
    # http://stackoverflow.com/a/11337310
    # http://linux.die.net/man/1/unbuffer
unbuffer roslaunch arlobot_launchers robot.launch --screen
