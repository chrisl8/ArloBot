SCRIPTDIR=$(cd $(dirname "$0") && pwd)
# Attempt to kill ROS if it is already running
echo "Killing everything, please wait a moment . . ."
if (pkill -f metatron_id.launch)
then
    while (pgrep -f metatron_id.launch)
    do
        echo "Waiting for Metatron to close . . ."
        sleep 1
    done
fi
if (pkill ngrok)
then
    while (pgrep ngrok)
    do
        echo "Waiting for ngrok to close . . ."
        sleep 1
    done
fi
if (pkill -f "arlobot_bringup minimal.launch")
then
    while (pgrep -f "arlobot_bringup minimal.launch")
    do
        echo "Waiting for Arlobot to close . . ."
        sleep 1
    done
fi
if (pkill roslaunch)
then
    while (pgrep roslaunch)
    do
        echo "Waiting for roslaunch to close . . ."
        sleep 1
    done
fi
if (pkill roscore)
then
    while (pgrep roscore)
    do
        echo "Waiting for roscore to close . . ."
        sleep 1
    done
fi
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

