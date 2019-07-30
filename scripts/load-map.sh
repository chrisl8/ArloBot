#!/usr/bin/env bash

MAP_FILE=${HOME}/.arlobot/rosmaps/${1}
if [[ $# -eq 0 ]]; then
  echo "You must provide a map file name."
  echo "Run listMaps.sh for a list of your maps."
  exit
fi

if [[ ! -f ${MAP_FILE} ]]; then
  MAP_FILE=${MAP_FILE}.yaml
fi
echo "${MAP_FILE}"
if [[ ! -f ${MAP_FILE} ]]; then
  echo "File does not exist!"
  exit 1
fi

if pgrep -f robot.launch; then
  if [[ $(jq '.hasScanseSweep' "${HOME}/.arlobot/personalDataForBehavior.json") == true ]]; then
    roslaunch arlobot_launchers load_map_wScanseSweep.launch map_file:="${MAP_FILE}"
  else
    roslaunch arlobot_launchers load_map.launch map_file:="${MAP_FILE}"
  fi
else
  echo "Robot must be running to start this."
  exit 1
fi
