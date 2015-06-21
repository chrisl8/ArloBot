#!/bin/bash
# This is the primary script to

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

# Start the entire robot
if ! (source ${SCRIPTDIR}/ros_prep.sh)
then
    echo "ROS Prep Failed, EXITING!"
    exit 1
fi
echo "Use kill_ros.sh to close."

# This allows the metatron_id.launch file to only load the joystick
# node if the joystick is present.
# I found that loading the joystick node on test systems with no joystick
# USB controller caused crashes in ROS.
# Rightly this should be a ROS parameter, but there is no facility within
# roslaunch to use params in if statements.
# Also, this should be in ros_prep.sh, but then the variable will not pass up
# to here unless I 'source' it, but ros_prep.sh is not written or tested
# to be sourced.
if [ $(jq '.hasXboxController' ${HOME}/.arlobot/personalDataForBehavior.json) == true ]
    then
    export HAS_XBOX_JOYSTICK=true
fi

# 'unbuffer' is required for running this from the node based 'behavior'
# scripts. Otherwise stdout data is buffered until ROS exits,
# which makes monitoring status impossible.
    # http://stackoverflow.com/a/11337310
    # http://linux.die.net/man/1/unbuffer
# NOTE: TODO: MAYBE? Seems inconsistent: Do not add "--screen" to the end of this. Doing that causes
# the process to not respond properly to the kill signals.
# If you need better insight into what ROS is doing, check the logs.
unbuffer roslaunch metatron_id metatron_id.launch --screen
