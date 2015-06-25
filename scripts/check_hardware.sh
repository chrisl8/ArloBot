#!/bin/bash
# This script checks that all required hardware is present

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

# USB Relay Controller
if [ $(jq '.useUSBrelay' ${HOME}/.arlobot/personalDataForBehavior.json) == true ]
    then
    ${SCRIPTDIR}/drcontrol.py -l|grep USB &> /dev/null
    if [ $? -gt 0 ]
        then
        echo "USB Relay Controller missing!"
        exit 1
    fi
fi

# Camera 0
if [ $(jq '.camera0' ${HOME}/.arlobot/personalDataForBehavior.json) == true ]
    then
    ls /dev/video0 &> /dev/null
    if [ $? -gt 0 ]
        then
        echo "Camera 0 Missing!"
        exit 1
    fi
fi

# Camera 1
if [ $(jq '.camera1' ${HOME}/.arlobot/personalDataForBehavior.json) == true ]
    then
    ls /dev/video1 &> /dev/null
    if [ $? -gt 0 ]
        then
        echo "Camera 1 Mising!"
        exit 1
    fi
fi

# Activity Board
if [ $(jq '.hasActivityBoard' ${HOME}/.arlobot/personalDataForBehavior.json) == true ]
    then
    ${SCRIPTDIR}/find_ActivityBoard.sh |grep USB &> /dev/null
    if [ $? -gt 0 ]
        then
        echo "Activity Board missing!"
        echo "If this is a test install with no Activity Board,"
        echo "edit ${HOME}/.arlobot/personalDataForBehavior.json"
        echo "and set 'hasActivityBoard' to false"
        exit 1
    fi
fi

# Quick Start Board
# While it should exist, it does not have to be plugged into the computer for Arlo to operate
#${SCRIPTDIR}/find_QuickStart.sh |grep USB &> /dev/null
#if [ $? -gt 0 ]
#then
#echo "Quick Start Board missing!"
#exit 1
#fi

# XV-11
if [ $(jq '.use_xv11' ${HOME}/.arlobot/personalDataForBehavior.json) == true ]
    then
    ${SCRIPTDIR}/find_XVLidar.sh |grep ACM &> /dev/null
    if [ $? -gt 0 ]
        then
        echo "XV-11 missing!"
        exit 1
    fi
    # XV-11 Ready
    ${SCRIPTDIR}/XVLidarStopMotor.sh
    if [ $? -gt 0 ]
        then
        echo "XV-11 not ready!"
        exit 1
    fi
fi
exit 0
