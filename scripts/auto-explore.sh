pgrep -f metatron_id.launch
if [ $? -eq 0 ]
then
echo "When you are done, save your map!"
echo "Please run './save-map.sh mapname' from another terminal when your map is done before closing this!"
roslaunch metatron_launchers add_autonomous_explore.launch
else
echo "Metatron must be running to start this."
fi

