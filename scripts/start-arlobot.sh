SCRIPTDIR=$(cd $(dirname "$0") && pwd)
${SCRIPTDIR}/ros_prep.sh
roslaunch arlobot_bringup minimal.launch --screen

