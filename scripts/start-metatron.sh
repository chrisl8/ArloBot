SCRIPTDIR=$(cd $(dirname "$0") && pwd)
# This is the primary script to
# Start the entire robot
if ! (${SCRIPTDIR}/ros_prep.sh)
then
    echo "ROS Prep Failed, EXITING!"
    exit 1
fi
# Start web server for Twilio
ngrok -authtoken E_-s84Q-fW2O1B_kU1Fl -subdomain=52c014b4 -log=stdout 8001 &
/opt/ros/indigo/bin/roslaunch metatron_id metatron_id.launch --screen &
echo "Use kill_ros.sh to close."

