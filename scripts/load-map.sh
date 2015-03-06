MAPFILE=${HOME}/.arlobot/rosmaps/${1}
if [ $# -eq 0 ]
then
    echo "You must provide a map file name."
    echo "Run list-maps.sh for a list of your maps."
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
    roslaunch metatron_launchers load_map.launch map_file:=${MAPFILE}
else
    echo "Metatron must be running to start this."
    exit 1
fi

