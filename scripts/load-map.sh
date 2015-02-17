if [ $# -eq 0 ]
then
    echo "You must provide a map file name."
    echo "Run list-maps.sh for a list of your maps."
    exit
fi

if [ -f ${1} ]
then
    MAPFILE=~/.arlobot/rosmaps/${1}.yaml
elif [ -f ~/.arlobot/rosmaps/${1}.yaml ]
then
    MAPFILE=${1}
else
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

