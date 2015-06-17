#!/bin/bash
# This script is meant to help set up your machine
# and user environment for Metatron and Arlobot to work

SCRIPTDIR=$(cd $(dirname "$0") && pwd)

# Check for node.js installation
if [ ! -e  ${HOME}/.nvm/nvm.sh ]
then
    echo "Installing Node Version Manager"
wget -qO- https://raw.githubusercontent.com/creationix/nvm/v0.25.4/install.sh | bash
fi

export NVM_DIR="${HOME}/.nvm"
[ -s "$NVM_DIR/nvm.sh" ] && . "$NVM_DIR/nvm.sh"  # This loads nvm
nvm install stable
nvm use stable

# Install required node packages
#echo "Installing required node packages . . ."
#echo "You may be asked for your password"
#echo "In order to run npm install -g"
#sudo npm install -g grunt forever

#echo "Fix personal npm folder permissions"
#sudo chown -R `whoami` ${HOME}/.npm/

echo "Grabbing dependencies for node packages."
cd ${SCRIPTDIR}/../node
npm install
cd ${SCRIPTDIR}

#if [ ! -d  ${SCRIPTDIR}/../node/node_modules/roslibjs ]
#then
#    echo "Installing roslibjs for arloBehavior.js"
#    echo "You may be asked for your password"
#    echo "in order to run sudo commands."
#    echo "Fix personal npm folder permissions"
#    sudo chown -R `whoami` ${HOME}/.npm/
#    cd ${SCRIPTDIR}/../node/node_modules/
#    git clone git@github.com:RobotWebTools/roslibjs.git
#    cd roslibjs
#    npm install
#fi

if [ ! -d  ${SCRIPTDIR}/../node/public/lcars/ ]
then
    echo "Cloning in lcars CSS Framework"
    cd ${SCRIPTDIR}/../node/public/
    git clone https://github.com/Garrett-/lcars.git
fi

# We will use ~/.arlobot to store "private" data
# That is data that doesn't need to be part of
# the public github repo like user tokens,
# sounds, and room maps and per robot settings
if [ ! -d ${HOME}/.arlobot ]
then
    mkdir ${HOME}/.arlobot
fi

ARLOHOME=${HOME}/.arlobot

for i in `ls ${SCRIPTDIR}/dotarlobot/`
do
    if [ -e  ${ARLOHOME}/${i} ]
    then
        diff -u ${SCRIPTDIR}/dotarlobot/${i} ${ARLOHOME}/${i}
    fi
    cp -i ${SCRIPTDIR}/dotarlobot/${i} ${ARLOHOME}/
done

if [ ! -d ${ARLOHOME}/rosmaps ]
then
    mkdir ${ARLOHOME}/rosmaps
fi

if [ ! -d ${ARLOHOME}/sounds ]
then
    mkdir ${ARLOHOME}/sounds
fi

if [ ! -d ${ARLOHOME}/status ]
then
    mkdir ${ARLOHOME}/status
fi
chmod -R 777 ${ARLOHOME}/status

# Install required Ubuntu packages
echo "Installing required Ubuntu packages . . ."
echo "You may be asked for your password"
echo "In order to run apt-get install."
# NOTE: You have to pipe /dev/null INTO apt-get to make it work from wget.
sudo apt-get install -qy ros-indigo-rosbridge-server imagemagick fswebcam festival festvox-en1 python-ftdi python-pip python-serial libv4l-dev jq unbuffer < /dev/null

# Install required Python packages
echo "Installing required Python packages . . ."
echo "You may be asked for your password"
sudo pip install twilio pylibftdi

# Install mjpg-streamer
if ! (which mjpg_streamer)
then
    cd ${SCRIPTDIR}
    svn co https://svn.code.sf.net/p/mjpg-streamer/code/ mjpg-streamer
    cd mjpg-streamer/mjpg-streamer
    # sudo apt-get install libv4l-dev # Installed earlier, left here for documentation of why
    make USE_LIBV4L2=true clean all
    sudo make install
    # mjpg_streamer usage example:
    #mjpg_streamer -i "/usr/local/lib/input_uvc.so -d /dev/video0 -f 30 -r 640x480" -o "/usr/local/lib/output_http.so -p 58180 -w ${SCRIPTDIR}/mjpg-streamer/mjpg-streamer/www"
fi

if ! (id|grep dialout>/dev/null)
then
    sudo adduser ${USER} dialout
    echo "You may have to reboot before you can use the Propeller Board."
fi

if ! [ -f /etc/udev/rules.d/99-libftdi.rules ]
then
    sudo ${SCRIPTDIR}/addRuleForUSBRelayBoard.sh
    echo "You may have to reboot before the USB Relay board will function!"
fi

if ! (grep mbrola /etc/festival.scm>/dev/null)
then
    echo "Updating default Festival voice"
    echo "You may be asked for your password"
    echo "To allow updates to /etc/festival.scm"
    sudo ${SCRIPTDIR}/updateFestivalDefaults.sh
fi

# Set up required sudo entries.
sudo -nl|grep resetUSB > /dev/null
if [ $? -ne 0 ]
then
    echo "Sudo entries already in place."
else
    echo "${USER} ALL = NOPASSWD: ${SCRIPTDIR}/resetUSB.sh" >> /tmp/arlobot_sudoers
    chmod 0440 /tmp/arlobot_sudoers
    sudo mv /tmp/arlobot_sudoers /etc/sudoers.d/
    sudo chown root:root /etc/sudoers.d/arlobot_sudoers
fi

if [ ${USER} == chrisl8 ]
then
    if ! [-f /home/robotStatusUser ]
    then
        echo "Adding robotStatusUser."
        echo "(This is NOT required for Arlobot, just a personal thing.)"
        sudo useradd -m robotStatusUser
        echo "Be sure to add your key to ~robotStatusUser/.ssh/authorized_keys"
        echo "for anyone who needs to use it!"
        echo "sudo su - robotStatusUser"
        echo "mkdir .ssh"
        echo "vim .ssh/authorized_keys"
        echo "(This is NOT required for Arlobot, just a personal thing.)"
    fi
    # This simulates the basement door being open,
    # which will cause the robot to stop.
    echo STOP > ${HOME}/.arlobot/status/room-MainFloorHome
fi

if ! [ -f ${HOME}/Desktop/arlobot.desktop ]
then
    echo "Modify and copy arlobot.desktop"
    echo "to your Desktop folder to create an icon"
    echo "in XWindows to start up the Robot!"
fi
