SCRIPTDIR=$(cd $(dirname "$0") && pwd)
echo "Activity Board should be sending a stead stream of:"
echo "i     0"
echo "i     0"
echo "etc."
echo "Paste this line into the terminal to get it to start sending sensor data:"
echo "d,0.403000,0.006760,0.0,0.0,0.0"
miniterm.py $(${SCRIPTDIR}/find_ActivityBoard.sh) 115200

