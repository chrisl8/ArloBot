if [[ $# -ne 1 ]]
then
echo 'Please provide a map name for saving to later!'
else
rosrun map_server map_saver -f ~/rosmaps/${1}
fi
