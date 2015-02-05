SCRIPTDIR=$(cd $(dirname "$0") && pwd)
if ! (${SCRIPTDIR}/ros_prep.sh)
then
    echo "ROS Prep Failed, EXITING!"
    exit 1
fi
roslaunch arlobot_bringup minimal.launch --screen

