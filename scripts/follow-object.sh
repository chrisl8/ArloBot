#!/usr/bin/env bash
pgrep -f robot.launch
if [[ $? -eq 0 ]]; then
  roslaunch arlobot_bringup follower.launch --screen
else
  echo "Robot must be running to start this."
fi
