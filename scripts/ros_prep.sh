SCRIPTDIR=$(cd $(dirname "$0") && pwd)
# Attempt to kill ROS if it is already running
${SCRIPTDIR}/kill_ros.sh
# If you want to be able to reset the ports,
# you need to add the script resetUSB.sh to the /etc/sudoers file,
# like this:
#chrisl8 ALL = NOPASSWD: /home/chrisl8/metatron/scripts/resetUSB.sh
sudo -nl|grep resetUSB > /dev/null
if [ $? -eq 0 ]
then
sudo -n ${SCRIPTDIR}/resetUSB.sh
sleep 3
fi
/opt/ros/indigo/bin/roscore &
sleep 3 # Give roscore time to start before attempting to set parameters
rosparam set /arlobot/port $(${SCRIPTDIR}/find_propeller.sh)
rosparam set /joystick/dev $(${SCRIPTDIR}/find_xbox_controller.sh)
rosparam set /camera1 $(${SCRIPTDIR}/find_camera.sh C615)
rosparam set /camera2 $(${SCRIPTDIR}/find_camera.sh HP)

