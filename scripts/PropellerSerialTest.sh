#!/bin/bash
# This script allows testing the Arlobot
# Propeller Board functions without
# ROS
# This is the first step in making sure your hardware
# is ready for ROS

# Modeled after ros_prep.sh - If ros_prep.sh has significant changes,
# this script may need updating also.

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

if (pgrep -f simpleide>/dev/null)
then
    echo "SimpleIDE is running,"
    echo "please close it and try again."
    exit 1
fi

# Check to make sure required hardware is present
if ! (${SCRIPTDIR}/check_hardware.sh)
then
    echo "ERROR: Hardware failed to come on line:"
    exit 1
fi

USB_PORT=$(${SCRIPTDIR}/find_ActivityBoard.sh)

python ${SCRIPTDIR}/../src/arlobot/arlobot_bringup/scripts/PropellerSerialTest.py ${USB_PORT}
