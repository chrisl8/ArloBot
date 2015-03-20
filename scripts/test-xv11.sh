SCRIPTDIR=$(cd $(dirname "$0") && pwd)
/opt/ros/indigo/bin/roscore &
while ! (rosparam list &> /dev/null)
do
    echo "Waiting for roscore to start . . ."
    sleep 1
done
rosrun xv_11_laser_driver neato_laser_publisher _port:=$(${SCRIPTDIR}/find_XVLidar.sh) _firmware_version:=2 &
roslaunch arlobot_rviz_launchers view_xv11.launch --screen
${SCRIPTDIR}/kill_ros.sh

