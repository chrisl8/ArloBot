#!/bin/bash
# This script is meant to help set up your machine
# and user environment for Metatron and Arlobot to work

SCRIPTDIR=$(cd $(dirname "$0") && pwd)

BLACK='\033[0;30m'
BLUE='\033[0;34m'
GREEN='\033[0;32m'
CYAN='\033[0;36m'
RED='\033[0;31m'
PURPLE='\033[0;35m'
ORANGE='\033[0;33m' # or brown
LIGHTGRAY='\033[0;37m'
DARKGRAY='\033[1;30m'
LIGHTBLUE='\033[1;34m'
LIGHTGREEN='\033[1;32m'
LIGHTCYAN='\033[1;36m'
LIGHTRED='\033[1;31m'
LIGHTPURPLE='\033[1;35m'
YELLOW='\033[1;33m'
WHITE='\033[1;37m'
NC='\033[0m' # NoColor

# Check for node.js installation
if [ ! -e  ${HOME}/.nvm/nvm.sh ]
    then
    printf "\n${YELLOW}[Installing Node Version Manager${NC}\n"
    wget -qO- https://raw.githubusercontent.com/creationix/nvm/v0.25.4/install.sh | bash
fi

export NVM_DIR="${HOME}/.nvm"
[ -s "$NVM_DIR/nvm.sh" ] && . "$NVM_DIR/nvm.sh"  # This loads nvm
nvm install stable
#nvm use stable # Redundant

# Install required node packages
#echo "Installing required node packages . . ."
#echo "You may be asked for your password"
#echo "In order to run npm install -g"
#sudo npm install -g grunt forever

#echo "Fix personal npm folder permissions"
#sudo chown -R `whoami` ${HOME}/.npm/

printf "\n${YELLOW}[Grabbing dependencies for node packages.]${NC}\n"
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

# Install required Ubuntu packages
printf "\n${YELLOW}[Installing additional Ubuntu packages for Metatron]${NC}\n"
# NOTE: You have to pipe /dev/null INTO apt-get to make it work from wget.
# expect-dev required to get 'unbuffer' which is required by node to spawn ROS commands and get real time stdout data
sudo apt-get install -qy ros-indigo-rosbridge-server imagemagick fswebcam festival festvox-en1 python-ftdi python-pip python-serial libv4l-dev jq expect-dev < /dev/null

# Install required Python packages
# pylibftdi is in the arlo setup script,
# and I don't think I use twilio in python anymore!
#printf "\n${YELLOW}[Installing required Python packages for Metatron]${NC}\n"
#sudo pip install twilio pylibftdi

printf "\n${YELLOW}[Installing mjpg_streamer for Web Page camera viewing]${NC}\n"
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

if [ ! -d  ${SCRIPTDIR}/../node/public/lcars/ ]
    then
    printf "\n${YELLOW}[Cloning in lcars CSS Framework]${NC}\n"
    cd ${SCRIPTDIR}/../node/public/
    git clone https://github.com/Garrett-/lcars.git
else
    printf "\n${YELLOW}[Updating lcars CSS Framework repo]${NC}\n"
    cd ${SCRIPTDIR}/../node/public/lcars
    git pull
fi

printf "\n${YELLOW}[Setting up .arlobot folder]${NC}\n"
printf "${BLUE}This holds personal data for your robot.${NC}\n"
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
        if ! (diff ${SCRIPTDIR}/dotarlobot/${i} ${ARLOHOME}/${i})
            then
            printf "\n${GREEN}The ${i} file in the repository is different from the one${NC}\n"
            printf "${GREEN}in your local settings.${NC}\n"
            printf "${GREEN}This is expected, but just in case, please look over the differences,${NC}\n"
            printf "${GREEN}and see if you need to copy in any new settings, or overwrite the file completely:${NC}\n"
            diff ${SCRIPTDIR}/dotarlobot/${i} ${ARLOHOME}/${i}
            cp -i ${SCRIPTDIR}/dotarlobot/${i} ${ARLOHOME}/
        fi
    else
        printf "\n"
        cp ${SCRIPTDIR}/dotarlobot/${i} ${ARLOHOME}/
        printf "${GREEN}A brand new ${RED}${ARLOHOME}/${i}${GREEN} file has been created,${NC}\n"
        printf "${RED}please edit this file to customize according to your robot!\n${NC}\n"
    fi
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

if ! (id|grep dialout>/dev/null)
    then
    printf "\n${GREEN}Adding your user to the 'dialout' group.${NC}\n"
    sudo adduser ${USER} dialout > /dev/null
    printf "${RED}You may have to reboot before you can use the Propeller Board.${NC}\n"
fi

if ! [ -f /etc/udev/rules.d/99-libftdi.rules ]
    then
    printf "\n${RED}Adding required sudo rule to reset USB ports.${NC}\n"
    sudo ${SCRIPTDIR}/addRuleForUSBRelayBoard.sh
    printf "${RED}You may have to reboot before the USB Relay board will function!${NC}\n"
fi

if ! (grep mbrola /etc/festival.scm>/dev/null)
    then
    printf "\n${YELLOW}[Updating default Festival voice]${NC}\n"
    printf "${BLUE}You may be asked for your password\nTo allow updates to /etc/festival.scm${NC}\n"
    sudo ${SCRIPTDIR}/updateFestivalDefaults.sh
fi

printf "\n${YELLOW}[Set up required sudo entries.]${NC}\n"
sudo -nl|grep resetUSB > /dev/null
if [ $? -ne 0 ]
    then
    printf "${BLUE}Sudo entries already in place.${NC}\n"
else
    echo "${USER} ALL = NOPASSWD: ${SCRIPTDIR}/resetUSB.sh" >> /tmp/arlobot_sudoers
    chmod 0440 /tmp/arlobot_sudoers
    sudo mv /tmp/arlobot_sudoers /etc/sudoers.d/
    sudo chown root:root /etc/sudoers.d/arlobot_sudoers
fi

if [ ${USER} == chrisl8 ]
    then
    if ! [ -d /home/robotStatusUser ]
        then
        printf "\n${YELLOW}[Adding robotStatusUser.${NC}\n"
        printf "${GREEN}(This is NOT required for Arlobot, just a personal thing.)${NC}\n"
        sudo useradd -m robotStatusUser
        printf "${GREEN}Be sure to add your key to ~robotStatusUser/.ssh/authorized_keys${NC}\n"
        printf "${GREEN}for anyone who needs to use it!${NC}\n"
        printf "${RED}sudo su - robotStatusUser${NC}\n"
        printf "${RED}mkdir .ssh${NC}\n"
        printf "${RED}vim .ssh/authorized_keys${NC}\n"
        printf "${GREEN}(This is NOT required for Arlobot, just a personal thing.)${NC}\n"
    fi
    # This simulates the basement door being open,
    # which will cause the robot to stop.
    echo STOP > ${HOME}/.arlobot/status/room-MainFloorHome
fi

if ! [ -f ${HOME}/Desktop/arlobot.desktop ]
    then
    printf "\n${GREEN}Modify and copy arlobot.desktop\nto your Desktop folder to create an icon\nin XWindows to start up the Robot!${NC}\n"
fi
