SCRIPTDIR=$(cd $(dirname "$0") && pwd)
${SCRIPTDIR}/ros_prep.sh
/opt/ros/indigo/bin/roslaunch metatron_id metatron_id.launch --screen

