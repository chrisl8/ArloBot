# Attempt to kill ROS if it is already running
echo "Killing everything, please wait a moment . . ."
pkill -f metatron_id.launch
pkill ngrok
sleep 10
pkill roslaunch
sleep 10
pkill roscore
sleep 1
echo "Everything Killed."

