#!/bin/bash
echo "If you do not have a map loaded,"
echo "set Global Options->Fixed Frame to"
echo "'odom' in order to make this work."
export ARLOBOT_MODEL=$(jq '.arlobotModel' ${HOME}/.arlobot/personalDataForBehavior.json | tr -d '"')'"')
roslaunch arlobot_rviz_launchers view_navigation.launch --screen

