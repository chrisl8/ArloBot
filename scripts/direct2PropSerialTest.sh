SCRIPTDIR=$(cd $(dirname "$0") && pwd)
if ! (which miniterm.py>/dev/null)
then
    sudo apt-get install python-serial
    echo "You may have to reboot before you can use the Propeller Board."
fi
if ! (id|grep dialout>/dev/null)
then
    sudo adduser ${USER} dialout
fi
echo "Activity Board should be sending a stead stream of:"
echo "i     0"
echo "i     0"
echo "etc."
echo "Paste this line into the terminal to get it to start sending sensor data:"
echo "d,0.403000,0.006760,0,0,0.0,0.0,0.0"
miniterm.py $(${SCRIPTDIR}/find_ActivityBoard.sh) 115200

