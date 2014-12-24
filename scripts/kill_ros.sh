SCRIPTDIR=$(cd $(dirname "$0") && pwd)
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
# If you want to be able to reset the ports,
# you need to add the script resetUSB.sh to the /etc/sudoers file,
# like this:
#chrisl8 ALL = NOPASSWD: /home/chrisl8/metatron/scripts/resetUSB.sh
sudo -nl|grep resetUSB > /dev/null
if [ $? -eq 0 ]
then
sudo -n ${SCRIPTDIR}/resetUSB.sh
fi

