#!/usr/bin/env bash
pgrep -f robot.launch
if [[ $? -eq 0 ]]; then
  echo "When you are done, save your map!"
  echo "Please run './save-map.sh mapname' from another terminal when your map is done before closing this!"
  if [[ $(jq '.use_xv11' ${HOME}/.arlobot/personalDataForBehavior.json) == true ]]; then
    roslaunch arlobot_navigation gmapping_demo_xv11DWAonly.launch
  else
    roslaunch arlobot_navigation gmapping_demo.launch
  fi
else
  echo "Robot must be running to start this."
fi
