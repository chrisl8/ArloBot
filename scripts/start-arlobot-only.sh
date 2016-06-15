#!/bin/bash
# Start just the basic ArloBot ROS setup.
# Use start-robot.sh to start EVERYTHING instead

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

if ! (${SCRIPTDIR}/ros_prep.sh)
then
    echo "ROS Prep Failed, EXITING!"
    exit 1
fi
export ARLOBOT_MODEL=$(jq '.arlobotModel' ${HOME}/.arlobot/personalDataForBehavior.json | tr -d '"')
roslaunch arlobot_bringup minimal.launch --screen

