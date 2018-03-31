#!/usr/bin/env bash
MAPFILE=${HOME}/.arlobot/rosmaps/${1}
if [ $# -eq 0 ]
then
    echo "You must provide a map file name."
    echo "Run listMaps.sh for a list of your maps."
    exit
fi

if [ ! -f ${MAPFILE} ]
then
    MAPFILE=${MAPFILE}.yaml
fi
echo ${MAPFILE}
if [ ! -f ${MAPFILE} ]
then
    echo "File does not exist!"
    exit 1
fi

pgrep -f robot.launch > /dev/null
if [ $? -eq 0 ]
then
    if [ $(jq '.hasScanseSweep' ${HOME}/.arlobot/personalDataForBehavior.json) == true  ]
    then
        roslaunch arlobot_launchers load_map_wScanseSweep.launch map_file:=${MAPFILE}
    else
        roslaunch arlobot_launchers load_map.launch map_file:=${MAPFILE}
    fi
else
    echo "Robot must be running to start this."
    exit 1
fi

