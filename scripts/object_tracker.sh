# This will run the code from R. Patrick Goebel's excellent ROS By Example book
# http://www.lulu.com/shop/r-patrick-goebel/ros-by-example-indigo-volume-1/paperback/product-22094373.html
# to "track" an object, that is, move left right to keep it in view, but not
# follow it around the room.
# Note that you must have Patrick's code cloned and imported into your system for this to work,
# and I suggest buying his book to learn how, although technically you could just get it from Github.
pgrep -f metatron_id.launch > /dev/null
if [ $? -eq 0 ]
then
    echo $DISPLAY|grep : > /dev/null
    if [ $? -eq 0 ]
    then
        roslaunch metatron_launchers object_tracker.launch
    else
        echo "This must be run from XWindows."
        exit 1
    fi
else
    echo "Metatron must be running to start this."
    exit 1
fi

