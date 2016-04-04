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

wrap_up_on_fail () {
    # USB Relay Controller
    if [ $(jq '.useUSBrelay' ${HOME}/.arlobot/personalDataForBehavior.json) == true ]
        then
        echo "Turning off all relays"
        ${SCRIPTDIR}/switch_relay_name.sh all off
    fi
    # Master Power Relay
    if [ $(jq '.useMasterPowerRelay' ${HOME}/.arlobot/personalDataForBehavior.json) == true ]
    then
        echo "Turning off Arlo Power."
        ${SCRIPTDIR}/switch_master_relay.sh off
    fi
    exit 1
}

# Turn on Arlo Power supply if "Master Relay" exists
if [ $(jq '.useMasterPowerRelay' ${HOME}/.arlobot/personalDataForBehavior.json) == true ]
    then
    echo "Turning on Arlo Power supply . . ."
    ${SCRIPTDIR}/switch_master_relay.sh on
    # Give Linux time to find the devices.
    sleep 1
fi

# USB Relay Controller
if [ $(jq '.useUSBrelay' ${HOME}/.arlobot/personalDataForBehavior.json) == true ]
    then
    ${SCRIPTDIR}/drcontrol.py -l|grep USB &> /dev/null
    if [ $? -gt 0 ]
        then
        echo "USB Relay Controller missing!"
        wrap_up_on_fail
    fi
fi

# Turn on five volt power supply if it exists
if [ $(jq '.relays.has_fiveVolt' ${HOME}/.arlobot/personalDataForBehavior.json) == true ]
    then
    echo "Turning on Five Volt power converter . . ."
    ${SCRIPTDIR}/switch_relay_name.sh fiveVolt on
    # Give Linux time to find the devices.
    sleep 3
fi

# Camera 0
if [ $(jq '.camera0' ${HOME}/.arlobot/personalDataForBehavior.json) == true ]
    then
    CAMERANAME=$(jq '.camera0name' ${HOME}/.arlobot/personalDataForBehavior.json | tr -d '"')
    VIDEODEVICE=$(${SCRIPTDIR}/find_camera.sh ${CAMERANAME})
    if [ $? -gt 0 ]
        then
        echo "Camera 0 missing!"
        wrap_up_on_fail
    fi
    ls ${VIDEODEVICE} &> /dev/null
    if [ $? -gt 0 ]
        then
        echo "Camera 0 missing!"
        wrap_up_on_fail
    fi
fi

# Camera 1
if [ $(jq '.camera1' ${HOME}/.arlobot/personalDataForBehavior.json) == true ]
    then
    CAMERANAME=$(jq '.camera1name' ${HOME}/.arlobot/personalDataForBehavior.json | tr -d '"')
    VIDEODEVICE=$(${SCRIPTDIR}/find_camera.sh ${CAMERANAME})
    if [ $? -gt 0 ]
        then
        echo "Camera 1 missing!"
        wrap_up_on_fail
    fi
    ls ${VIDEODEVICE} &> /dev/null
    if [ $? -gt 0 ]
        then
        echo "Camera 1 missing!"
        wrap_up_on_fail
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
        wrap_up_on_fail
    fi
fi

# Quick Start Board
# While it should exist, it does not have to be plugged into the computer for Arlo to operate
#${SCRIPTDIR}/find_QuickStart.sh |grep USB &> /dev/null
#if [ $? -gt 0 ]
#then
#echo "Quick Start Board missing!"
#wrap_up_on_fail
#fi

# XV-11
if [ $(jq '.use_xv11' ${HOME}/.arlobot/personalDataForBehavior.json) == true ]
    then
    ${SCRIPTDIR}/find_XVLidar.sh |grep ACM &> /dev/null
    if [ $? -gt 0 ]
        then
        echo "XV-11 missing!"
        wrap_up_on_fail
    fi
fi
exit 0
