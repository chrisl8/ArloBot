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

pgrep -f metatron_id.launch
if [ $? -eq 0 ]
then
    if [ $(jq '.use_xv11' ${HOME}/.arlobot/personalDataForBehavior.json) == true  ]
    then
        roslaunch metatron_launchers load_map_xv11.launch map_file:=${MAPFILE}
    else
        roslaunch metatron_launchers load_map.launch map_file:=${MAPFILE}
    fi
else
    echo "Metatron must be running to start this."
    exit 1
fi

