SCRIPTDIR=$(cd $(dirname "$0") && pwd)
if (pgrep -f simpleide>/dev/null)
then
    echo "SimpleIDE is running,"
    echo "please close it and try again."
    exit 1
fi
# Attempt to kill ROS if it is already running and reset USB Ports
${SCRIPTDIR}/kill_ros.sh
echo "Clearing ROS Logs . . ."
rm -r ${HOME}/.ros/log/*
# Check to make sure required hardware is present:
# This will wait 5 seconds for the USB reset to finish,
# before exiting to state that something is missing.
echo "Waiting for USB ports to come on line . . ."
hardwareCheckSeconds=0
hardwareTimeout=10
while ! (${SCRIPTDIR}/check_hardware.sh &> /dev/null)
do
    hardwareCheckSeconds=$((hardwareCheckSeconds+1))
    # Check for timeout BEFORE we sleep.
    if [ ${hardwareCheckSeconds} -eq ${hardwareTimeout} ]
    then
        echo "ERROR: Hardware failed to come on line:"
        # Run again to display the error on the screen:
        ${SCRIPTDIR}/check_hardware.sh
        exit 1
    fi
    echo "Waiting for USB ports to come on line . . ."
    sleep 1
done
${SCRIPTDIR}/XVLidarStartMotor.sh
/opt/ros/indigo/bin/roscore &
while ! (rosparam list &> /dev/null)
do
    echo "Waiting for roscore to start . . ."
    sleep 1
done
if [ ! -d ${HOME}/.arlobot/status/ ]
then
    mkdir ${HOME}/.arlobot/status/
fi
chmod 777 ${HOME}/.arlobot/status/ &> /dev/null
rosparam set /arlobot/port $(${SCRIPTDIR}/find_ActivityBoard.sh)
rosparam set /xv11/port $(${SCRIPTDIR}/find_XVLidar.sh)
rosparam set /joystick/dev $(${SCRIPTDIR}/find_xbox_controller.sh)
rosparam set /camera1 $(${SCRIPTDIR}/find_camera.sh C615)
rosparam set /camera2 $(${SCRIPTDIR}/find_camera.sh HP)
if [ $(jq '.wait_for_door_confirmation' ${HOME}/.arlobot/personalDataForBehavior.json) == true   ]
then
    echo "Open and close the basement door to ensure lockout is working."
    echo STOP > ${HOME}/.arlobot/status/room-MainFloorHome
fi
exit 0
