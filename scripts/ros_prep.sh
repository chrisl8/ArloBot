SCRIPTDIR=$(cd $(dirname "$0") && pwd)
# Attempt to kill ROS if it is already running and reset USB Ports
${SCRIPTDIR}/kill_ros.sh
/opt/ros/indigo/bin/roscore &
while ! (rosparam list &> /dev/null)
do
    echo "Waiting for roscore to start . . ."
    sleep 1
done
rosparam set /arlobot/port $(${SCRIPTDIR}/find_ActivityBoard.sh)
rosparam set /joystick/dev $(${SCRIPTDIR}/find_xbox_controller.sh)
rosparam set /camera1 $(${SCRIPTDIR}/find_camera.sh C615)
rosparam set /camera2 $(${SCRIPTDIR}/find_camera.sh HP)

