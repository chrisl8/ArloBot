# Check that all required hardware is present
SCRIPTDIR=$(cd $(dirname "$0") && pwd)
# USB Relay Controller
${SCRIPTDIR}/drcontrol.py -l|grep USB>/dev/null
if [ $? -gt 0 ]
then
echo "USB Relay Controller missing!"
exit 1
fi
# Camera 0
ls /dev/video0 > /dev/null
if [ $? -gt 0 ]
then
echo "Camera 0 Missing!"
exit 1
fi
# Camera 1
ls /dev/video1 > /dev/null
if [ $? -gt 0 ]
then
echo "Camera 1 Mising!"
exit 1
fi
# Activity Board
${SCRIPTDIR}/find_ActivityBoard.sh |grep USB > /dev/null
if [ $? -gt 0 ]
then
echo "Activity Board missing!
exit 1
fi
# Quick Start Board
${SCRIPTDIR}/find_QuickStart.sh |grep USB > /dev/null
if [ $? -gt 0 ]
then
echo "Quick Start Board missing!
exit 1
fi
exit 0
