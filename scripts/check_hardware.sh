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
    # Turn it off if it is already one
    if [ "$(${SCRIPTDIR}/switch_master_relay.sh read)" == "on" ]
        then
        ${SCRIPTDIR}/switch_master_relay.sh off
        sleep 2
    fi
    ${SCRIPTDIR}/switch_master_relay.sh on
    # Give Linux time to find the devices.
    echo "Giving it 1 second to come online . . ."
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
    # Experience shows any less than 5 seconds causes some devices to fail.
    USBDELAYTIME=5
    while [ ${USBDELAYTIME} -gt 0 ]
    do
        echo "Giving USB devices ${USBDELAYTIME} seconds to come online . . ."
        sleep 1
        USBDELAYTIME=$((USBDELAYTIME-1))
    done
fi

echo "Checking all configured devices to make sure they are available . . ."

# Camera 0
if [ $(jq '.camera0' ${HOME}/.arlobot/personalDataForBehavior.json) == true ]
    then
    echo "Checking Camera 0 . . ."
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
    echo "Checking Camera 1 . . ."
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

# Joystick
# Changes for Ubuntu 16.04:
# xpad driver no longer loads itself at boot. Not sure why not.
# We no longer get the four "empty" js devices in /dev/input/ for js0 to js4 any time the wireless USB receiver is plugged in
# Instaed /dev/input/js0 only appears the moment the actual wireless joystick is truned on!
# NOTE: I tried xboxdrv and it was a nightmare. Unless I had the actual xbox controller powered on when I started it,
# it would not recognize the controller and any time I power cyclced the controller it changed device locations!
if [ $(jq '.hasXboxController' ${HOME}/.arlobot/personalDataForBehavior.json) == true  ]
then
    echo "Checking Xbox Controller . . ."
    # 1. Bring up the xpad driver after the USB power is on.
    # This doesn't hurt anything if it is already on.
    sudo modprobe xpad
    # Since we don't get the four "empty" js devices, we just tag this to js0,
    # since we won't really see it until the controller is turned on,
    # but I do NOT want to require the controller to be on before starting ROS.
    # I often think to grab it while ROS is already operating.
    JOYSTICKDEVICE=/dev/input/js0
    # So now this code is junk, unless the driver changes its behavior again.
    #JOYSTICKDEVICE=$(${SCRIPTDIR}/find_xbox_controller.sh)
    #if [ -z ${JOYSTICKDEVICE} ]
    #then
    #    echo "Joystick missing!"
    #    wrap_up_on_fail
    #fi
fi

# Activity Board
if [ $(jq '.hasActivityBoard' ${HOME}/.arlobot/personalDataForBehavior.json) == true ]
    then
    echo "Checking Activity Board . . ."
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
if [ $(jq '.hasActivityBoard' ${HOME}/.arlobot/personalDataForBehavior.json) == true ]
    then
    echo "Checking Quick Start Board . . ."
    ${SCRIPTDIR}/find_QuickStart.sh |grep USB &> /dev/null
    if [ $? -gt 0 ]
        then
        echo "Quick Start Board missing!"
        wrap_up_on_fail
    fi
fi

# XV-11
if [ $(jq '.use_xv11' ${HOME}/.arlobot/personalDataForBehavior.json) == true ]
    then
        echo "Checking XV11"
    ${SCRIPTDIR}/find_XVLidar.sh |grep ACM &> /dev/null
    if [ $? -gt 0 ]
        then
        echo "XV-11 missing!"
        wrap_up_on_fail
    fi
fi

echo "Hardware Check SUCCESS! All devices found."

exit 0
