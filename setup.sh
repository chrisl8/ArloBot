if ! (id|grep dialout>/dev/null)
then
    sudo adduser ${USER} dialout
    echo "You may have to reboot before you can use the Propeller Board."
fi
sudo apt-get install python-ftdi python-pip python-serial
sudo pip install pylibftdi

if ! [ -f /etc/udev/rules.d/99-libftdi.rules ]
then
    sudo addRuleForUSBRelayBoard.sh
    echo "You may have to reboot before the USB Relay board will function!"
fi

