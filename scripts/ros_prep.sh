SCRIPTDIR=$(cd $(dirname "$0") && pwd)
# Attempt to kill ROS if it is already running and reset USB Ports
${SCRIPTDIR}/kill_ros.sh
/opt/ros/indigo/bin/roscore &
sleep 3 # Give roscore time to start before attempting to set parameters
rosparam set /arlobot/port $(${SCRIPTDIR}/find_ActivityBoard.sh)
rosparam set /joystick/dev $(${SCRIPTDIR}/find_xbox_controller.sh)
rosparam set /camera1 $(${SCRIPTDIR}/find_camera.sh C615)
rosparam set /camera2 $(${SCRIPTDIR}/find_camera.sh HP)

