SCRIPTDIR=$(cd $(dirname "$0") && pwd)
# Attempt to kill ROS if it is already running and reset USB Ports
${SCRIPTDIR}/kill_ros.sh
# Check to make sure required hardware is present:
# This will wait 5 seconds for the USB reset to finish,
# before exiting to state that something is missing.
echo "Waiting for USB ports to come on line . . ."
hardwareCheckSeconds=0
hardwareTimeout=5
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
if [ ! -d ${HOME}/arloStatus ]
then
    mkdir ${HOME}/arloStatus
fi
chmod 777 ${HOME}/arloStatus &> /dev/null
rosparam set /arlobot/port $(${SCRIPTDIR}/find_ActivityBoard.sh)
rosparam set /xv11/port $(${SCRIPTDIR}/find_XVLidar.sh)
rosparam set /joystick/dev $(${SCRIPTDIR}/find_xbox_controller.sh)
rosparam set /camera1 $(${SCRIPTDIR}/find_camera.sh C615)
rosparam set /camera2 $(${SCRIPTDIR}/find_camera.sh HP)
exit 0
