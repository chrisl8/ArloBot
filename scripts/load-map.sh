if [ $# -eq 0 ]
then
echo "You must provide a map file name."
echo "Run list-maps.sh for a list of your maps."
exit
fi
pgrep -f metatron_id.launch
if [ $? -eq 0 ]
then
roslaunch metatron_launchers load_map.launch map_file:=${1}
else
echo "Metatron must be running to start this."
fi

