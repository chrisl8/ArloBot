# This script is meant to help set up your machine
# and user environment for Metatron and Arlobot to work

SCRIPTDIR=$(cd $(dirname "$0") && pwd)

# Check for node.js installation
if ! (which node>/dev/null)
then
    echo "You do not have node.js installed."
    echo "Please run the following commands to install node.js,"
    echo "or use instructions from http://nodejs.org/ if you prefer,"
    echo "and then run this setup script again."
    echo ""
    echo "cd"
    echo "# Currently roslib does not work with node 0.12.0"
    echo "wget http://nodejs.org/dist/v0.10.35/node-v0.10.35.tar.gz"
    echo "tar xzf node-v0.10.35.tar.gz"
    echo "cd node-v0.10.35"
    echo "./configure"
    echo "make"
    echo "sudo make install"
    echo "# And you had better install roslib by hand too!"
    echo "cd ~/metatron/behavior"
    echo "npm install roslib"
    exit 1
fi

# Install required node packages
echo "Installing required node packages . . ."
echo "You may be asked for your password"
echo "In order to run npm install -g"
sudo npm install -g grunt

echo "Fix personal npm folder permissions"
sudo chown -R `whoami` ${HOME}/.npm/

echo "Grabbing dependencies for node packages."
cd ${SCRIPTDIR}/../node/webserver
npm install
cd ${SCRIPTDIR}/../node
npm install
cd ${SCRIPTDIR}

if [ ! -d  ${SCRIPTDIR}/../node/node_modules/roslibjs ]
then
    echo "Installing roslibjs for arloBehavior.js"
    echo "You may be asked for your password"
    echo "in order to run sudo commands."
    echo "Fix personal npm folder permissions"
    sudo chown -R `whoami` ${HOME}/.npm/
    cd ${SCRIPTDIR}/../node/node_modules/
    git clone git@github.com:RobotWebTools/roslibjs.git
    cd roslibjs
    npm install
fi

# We will use ~/.arlobot to store "private" data
# That is data that doesn't need to be part of
# the public github repo like user tokens,
# sounds, and room maps
if [ ! -d ${HOME}/.arlobot ]
then
    mkdir ${HOME}/.arlobot
fi

ARLOHOME=${HOME}/.arlobot

for i in `ls ${SCRIPTDIR}/dotarlobot/`
do
    if [ -e  ${ARLOHOME}/${i} ]
    then
        git diff ${SCRIPTDIR}/dotarlobot/${i} ${ARLOHOME}/${i}
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
sudo apt-get install ros-indigo-rosbridge-server imagemagick fswebcam festival festvox-en1 python-ftdi python-pip python-serial libv4l-dev jq

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

echo "Chris, did you add the robotStatusUser user,"
echo "and his key?"
echo "(This is NOT required for Arlobot, just a personal thing.)"
#sudo useradd -m robotStatusUser
#sudo su - robotStatusUser
#mkdir .ssh
#vim .ssh/authorized_keys
echo STOP > ${HOME}/.arlobot/status/room-MainFloorHome
if ! [ -f ${HOME}/Desktop/arlobot.desktop ]
then
    echo "Modify and copy arlobot.desktop"
    echo "to your Desktop folder to create an icon"
    echo "in XWindows to start up the Robot!"
fi

