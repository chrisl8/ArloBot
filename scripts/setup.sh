#!/bin/bash
# This script is meant to help set up your machine
# and user environment for Metatron and Arlobot to work

# Grab and save the path to this script
# http://stackoverflow.com/a/246128
SOURCE="${BASH_SOURCE[0]}"
while [ -h "$SOURCE" ]; do # resolve $SOURCE until the file is no longer a symlink
  DIR="$( cd -P "$( dirname "$SOURCE" )" && pwd )"
  SOURCE="$(readlink "$SOURCE")"
  [[ $SOURCE != /* ]] && SOURCE="$DIR/$SOURCE" # if $SOURCE was a relative symlink, we need to resolve it relative to the path where the symlink file was located
done
SCRIPTDIR="$( cd -P "$( dirname "$SOURCE" )" && pwd )"
# echo ${SCRIPTDIR} # For debugging

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

if [ ! -e  ${HOME}/.nvm/nvm.sh ]
    then
    printf "\n${YELLOW}[Installing Node Version Manager${NC}\n"
    wget -qO- https://raw.githubusercontent.com/creationix/nvm/v0.25.4/install.sh | bash
fi

printf "\n${YELLOW}[Initializing Latest Stable Node version]${NC}\n"
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

printf "\n${YELLOW}[Grabbing dependencies for node packages]${NC}\n"
cd
if ! (which forever > /dev/null)
    then
    npm install -g forever
fi
if ! (which log.io-harvester > /dev/null)
    then
    npm install -g log.io
fi
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
# All required Ubuntu packages have been moved to the Arlobot setup.sh file,
# in order to consolidate and avoid running apt-get twice.
#printf "\n${YELLOW}[Installing additional Ubuntu packages for Metatron]${NC}\n"
# NOTE: You have to pipe /dev/null INTO apt-get to make it work from wget.
#sudo apt-get install -qy ros-indigo-rosbridge-server imagemagick fswebcam festival festvox-en1 python-ftdi python-pip python-serial libv4l-dev jq expect-dev < /dev/null

# Install required Python packages
# pylibftdi is in the arlo setup script,
# and I don't think I use twilio in python anymore!
#printf "\n${YELLOW}[Installing required Python packages for Metatron]${NC}\n"
#sudo pip install twilio pylibftdi

if ! (which mjpg_streamer > /dev/null)
    then
    printf "\n${YELLOW}[Installing mjpg_streamer for Web Page camera viewing]${NC}\n"
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
    pwd
    git clone https://github.com/Garrett-/lcars.git
else
    printf "\n${YELLOW}[Updating lcars CSS Framework repo]${NC}\n"
    cd ${SCRIPTDIR}/../node/public/lcars
    git pull
fi

if ! [ -f ${HOME}/Desktop/arlobot.desktop ]
    then
    printf "\n${YELLOW}[Creating Desktop Icon]${NC}\n"
    echo "[Desktop Entry]" > ${HOME}/Desktop/arlobot.desktop
    echo "Encoding=UTF-8" >> ${HOME}/Desktop/arlobot.desktop
    echo "Name=ArloBot" >> ${HOME}/Desktop/arlobot.desktop
    echo "GenericName=ArloBot" >> ${HOME}/Desktop/arlobot.desktop
    echo "Comment=Start the robot" >> ${HOME}/Desktop/arlobot.desktop
    if (which lxterminal)
        then
        echo "Exec=lxterminal --command \"${HOME}/catkin_ws/src/Metatron/scripts/arlobotXwindows.sh\"" >> ${HOME}/Desktop/arlobot.desktop
    elif (which gnome-terminal)
        then
        echo "Exec=gnome-terminal --command \"${HOME}/catkin_ws/src/Metatron/scripts/arlobotXwindows.sh\"" >> ${HOME}/Desktop/arlobot.desktop
    fi
    echo "Icon=${HOME}/catkin_ws/src/Metatron/node/public/icons/mstile-70x70.png" >> ${HOME}/Desktop/arlobot.desktop
    echo "Type=Application" >> ${HOME}/Desktop/arlobot.desktop
    echo "Path=${HOME}/catkin_ws/src/Metatron/scripts/" >> ${HOME}/Desktop/arlobot.desktop
    echo "Terminal=false" >> ${HOME}/Desktop/arlobot.desktop
    chmod +x ${HOME}/Desktop/arlobot.desktop
fi

if [ ! -d ${HOME}/.arlobot ]
    then
    printf "\n${YELLOW}[Setting up .arlobot folder]${NC}\n"
    printf "${GREEN}This holds personal data for your robot.${NC}\n"
    # We will use ~/.arlobot to store "private" data
    # That is data that doesn't need to be part of
    # the public github repo like user tokens,
    # sounds, and room maps and per robot settings
    mkdir ${HOME}/.arlobot
fi

ARLOHOME=${HOME}/.arlobot

for i in `ls ${SCRIPTDIR}/dotarlobot/`
do
    if [ -e  ${ARLOHOME}/${i} ]
        then
        if ! (diff ${SCRIPTDIR}/dotarlobot/${i} ${ARLOHOME}/${i} > /dev/null)
            then
            printf "\n${GREEN}The ${RED}${i}${GREEN} file in the repository is different from the one${NC}\n"
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
        printf "${LIGHTPURPLE}Please edit this file to customize according to your robot!${NC}\n"
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

if ! [ -f /etc/udev/rules.d/99-libftdi.rules ]
    then
    printf "\n${YELLOW}[Adding required sudo rule to reset USB ports.]${NC}\n"
    sudo ${SCRIPTDIR}/addRuleForUSBRelayBoard.sh
    printf "${RED}You may have to reboot before the USB Relay board will function!${NC}\n"
fi

if ! (grep mbrola /etc/festival.scm>/dev/null)
    then
    sudo ${SCRIPTDIR}/updateFestivalDefaults.sh
fi

if ! (sudo -nl|grep resetUSB > /dev/null)
    then
    printf "\n${YELLOW}[Setting up required sudo entries.]${NC}\n"
    echo "${USER} ALL = NOPASSWD: ${SCRIPTDIR}/resetUSB.sh" >> /tmp/arlobot_sudoers
    chmod 0440 /tmp/arlobot_sudoers
    sudo chown root:root /tmp/arlobot_sudoers
    sudo mv /tmp/arlobot_sudoers /etc/sudoers.d/
    sudo chown root:root /etc/sudoers.d/arlobot_sudoers
fi

if [ ${USER} == chrisl8 ]
    then
    if ! [ -d /home/robotStatusUser ]
        then
        printf "\n${YELLOW}[Adding robotStatusUser.]${NC}\n"
        printf "${GREEN}(This is NOT required for Arlobot, just a personal thing.)${NC}\n"
        sudo useradd -m robotStatusUser
        printf "${GREEN}Be sure to add your key to ~robotStatusUser/.ssh/authorized_keys${NC}\n"
        printf "${GREEN}for anyone who needs to use it!${NC}\n"
        printf "${RED}sudo su - robotStatusUser${NC}\n"
        printf "${RED}mkdir .ssh${NC}\n"
        printf "${RED}vim .ssh/authorized_keys${NC}\n"
        printf "${GREEN}(This is NOT required for Arlobot, just a personal thing.)${NC}\n"
    fi
fi
