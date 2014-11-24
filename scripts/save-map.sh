if [[ $# -ne 1 ]]
then
echo 'Please provide a map name on the command line.'
else
rosrun map_server map_saver -f ~/rosmaps/${1}
fi

