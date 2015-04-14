if ! (id|grep dialout>/dev/null)
then
    echo "Adding your user to the dialout group,"
    echo "You may be asked for your password."
    sudo adduser ${USER} dialout
    echo "You may have to reboot before you can use the Propeller Board."
fi

echo "Installing required Ubuntu packages . . ."
echo "You may be asked for your password"
echo "In order to run apt-get install."
sudo apt-get install python-ftdi python-pip python-serial ros-indigo-openni-* ros-indigo-openni2-* \
    ros-indigo-freenect-* ros-indigo-vision-opencv libopencv-dev python-opencv
sudo pip install pylibftdi

if ! [ -f /etc/udev/rules.d/99-libftdi.rules ]
then
    echo "Adding required sudo rule to reset USB ports."
    echo "You may be asked for your password."
    sudo addRuleForUSBRelayBoard.sh
    echo "You may have to reboot before the USB Relay board will function!"
fi

rospack profile

