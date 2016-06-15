#!/usr/bin/env bash
pgrep -f robot.launch > /dev/null
if [ $? -eq 0 ]
then
    echo "When you are done, save your map!"
    echo "Please run './save-map.sh mapname' from another terminal when your map is done before closing this!"
    if [ $(jq '.use_xv11' ${HOME}/.arlobot/personalDataForBehavior.json) == true  ]
    then
        roslaunch arlobot_launchers add_autonomous_explore_xv11.launch
    else
        roslaunch arlobot_launchers add_autonomous_explore.launch
    fi
else
    echo "Metatron must be running to start this."
fi

