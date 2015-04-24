SCRIPTDIR=$(cd $(dirname "$0") && pwd)
# This is the primary script to
# Start the entire robot
if ! (${SCRIPTDIR}/ros_prep.sh)
then
    echo "ROS Prep Failed, EXITING!"
    exit 1
fi
nohup /opt/ros/indigo/bin/roslaunch metatron_id metatron_id.launch &
echo "Use kill_ros.sh to close."
exit 0

