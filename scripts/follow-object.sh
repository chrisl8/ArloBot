#!/usr/bin/env bash
pgrep -f metatron.launch
if [ $? -eq 0 ]
then
roslaunch arlobot_bringup follower.launch --screen
else
echo "Metatron must be running to start this."
fi

