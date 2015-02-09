SCRIPTDIR=$(cd $(dirname "$0") && pwd)
# Call the USB Reset Script properly
# If you want to be able to reset the ports,
# you need to add the script resetUSB.sh to the /etc/sudoers file,
# like this:
#chrisl8 ALL = NOPASSWD: /home/chrisl8/metatron/scripts/resetUSB.sh
sudo -nl|grep resetUSB > /dev/null
if [ $? -eq 0 ]
then
sudo -n ${SCRIPTDIR}/resetUSB.sh
fi

