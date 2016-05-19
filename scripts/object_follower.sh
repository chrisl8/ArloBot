#!/bin/bash
# This will run the code from R. Patrick Goebel's excellent ROS By Example book
# http://www.lulu.com/shop/r-patrick-goebel/ros-by-example-indigo-volume-1/paperback/product-22094373.html
# to FOLLOW an oject around the room.
# Note that you must have Patrick's code cloned and imported into your system for this to work,
# and I suggest buying his book to learn how, although technically you could just get it from Github.
pgrep -f metatron_id.launch > /dev/null
if [ $? -eq 0 ]
    then
    # Set up ROS Environment
    export ROS_HOSTNAME=`uname -n`
    export ROS_MASTER_URI=http://localhost:11311
    export ROSLAUNCH_SSH_UNKNOWN=1
    source ~/catkin_ws/devel/setup.bash
    export DISPLAY=:0
    unbuffer roslaunch metatron_launchers object_follower.launch --screen
else
    echo "Metatron must be running to start this."
    exit 1
fi
