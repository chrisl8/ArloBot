SCRIPTDIR=$(cd $(dirname "$0") && pwd)
sudo -nl|grep resetUSB > /dev/null
if [ $? -eq 0 ]
then
sudo -n ${SCRIPTDIR}/resetUSB.sh
else
    echo "Run setup.sh to add the required SUDO entries!"
    exit 1
fi

