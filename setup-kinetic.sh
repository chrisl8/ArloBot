#!/bin/bash
# ROS Arlobot Automated Install

ROS_VERSION=kinetic

# Run this straight off of github like this:
# bash <(wget -qO- --no-cache https://raw.githubusercontent.com/chrisl8/ArloBot/new-serial-interface/setup-kinetic.sh)

set -e

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
LIGHT_RED='\033[1;31m'
LIGHT_PURPLE='\033[1;35m'
YELLOW='\033[1;33m'
WHITE='\033[1;37m'
NC='\033[0m' # NoColor

printf "\n${YELLOW}SETTING UP ROS ${ROS_VERSION} FOR YOUR ARLOBOT!${NC}\n"
printf "${YELLOW}---------------------------------------------------${NC}\n"
printf "${GREEN}You will be asked for your password for running commands as root!${NC}\n"

version=`lsb_release -sc`

printf "\n${YELLOW}[Checking the Ubuntu version]${NC}\n"
printf "${BLUE}Ubuntu ${version} found${NC}\n"
case ${version} in
  "xenial")
;;
*)
printf "${RED}[This script will only work on Ubuntu Xenial (16.04)]${NC}\n"
exit 1
esac

# I never use this, but if you are having time issues maybe uncomment this.
#printf "${YELLOW}[Installing chrony and setting the ntpdate]${NC}\n"
#sudo apt-get install -y chrony
#sudo ntpdate ntp.ubuntu.com

if ! [[ -e /etc/apt/sources.list.d/ros-latest.list ]]; then
    printf "${YELLOW}[Adding the ROS repository]${NC}\n"
    # This should follow the official ROS install instructions closely.
    # That is why there is a separate section for extra packages that I need for Arlo.
    sudo sh -c "echo \"deb http://packages.ros.org/ros/ubuntu ${version} main\" > /etc/apt/sources.list.d/ros-latest.list"
    printf "${BLUE}[Checking the ROS keys]${NC}\n"
    if ! apt-key list | grep -i "ROS builder"; then
        printf "${BLUE}[Adding the ROS keys]${NC}\n"
        sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
        printf "${BLUE}^^ He says it is 'OK'.${NC}\n"
    fi
fi

printf "\n${YELLOW}[Updating & upgrading all existing Ubuntu packages]${NC}\n"
sudo apt update
sudo apt upgrade -y

# This should follow the official ROS install instructions closely.
# That is why there is a separate section for extra packages that I need for Arlo.
if ! (dpkg -s ros-${ROS_VERSION}-desktop-full|grep "Status: install ok installed" &> /dev/null); then
    printf "\n${YELLOW}[Installing ROS]${NC}\n"
    sudo apt install -y ros-${ROS_VERSION}-desktop-full
    printf "${YELLOW}[ROS installed!]${NC}\n"
    printf "\n${YELLOW}[rosdep init and python-rosinstall]${NC}\n"
    if ! [[ -e /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
        sudo sh -c "rosdep init"
    fi
    printf "${BLUE}Running rosdep update . . .${NC}\n"
    rosdep update
    source /opt/ros/${ROS_VERSION}/setup.bash
    printf "${BLUE}Installing python-rosinstall:${NC}\n"
    sudo apt install -y python-rosinstall
    # END Offical ROS Install section
fi

# In case .bashrc wasn't set up, or you didn't reboot
if ! (which catkin_make > /dev/null); then
    source /opt/ros/${ROS_VERSION}/setup.bash
fi

printf "\n${YELLOW}[Installing additional Ubuntu and ROS Packages for Arlo]${NC}\n"
printf "${BLUE}This runs every time, in case new packages were added.${NC}\n"
# Notes on what the packages are for:

# build-essential - Needed for building almost anything from source
# python-serial is required for ROS to talk to the Propeller board

# For 8-CH USB Relay board:
# Reference: https://code.google.com/p/drcontrol/wiki/Install_RaspberryPi">https://code.google.com/p/drcontrol/wiki/Install_RaspberryPi
# python-ftdi,  python-pip and sudo pip install pylibftdi
# TEST:
#python -m pylibftdi.examples.list_devices
#Should return:
#FTDI:FT245R USB FIFO:A9026EI5
#If you have a USB Relay board attached via USB.

# expect-dev required to get 'unbuffer' which is required by node to spawn ROS commands and get real time stdout data
    # http://stackoverflow.com/a/11337310
    # http://linux.die.net/man/1/unbuffer
# jq allows shell scripts to read .json formatted config files.
# festival and fsetvox-en1 are for text to speech
#libav-tools is for ffmpeg to stream audio to another system.
#zbar-tools for reading QR codes.
#libftdi1 is required by SimpleIDE for the Parallax Propeller board
#libgif-dev is required for roslib in order to build canvas
#rtabmap is for 3D mapping
#pulseaudio pavucontrol are for setting the default microphone. I use this for mycroft among other things
#ros-${ROS_VERSION}-pointcloud-to-laserscan for Scanse Sweep
#ros-${ROS_VERSION}-geodesy for hector compile, might not need it if I stop using hector_explore
#libceres-dev for hector compile, might not need it if I stop using hector_explore
#git allows for cloning of repositories

sudo apt install -y build-essential ros-${ROS_VERSION}-rqt-* ros-${ROS_VERSION}-kobuki-ftdi python-ftdi1 python-pip python-serial ros-${ROS_VERSION}-openni-* ros-${ROS_VERSION}-openni2-* ros-${ROS_VERSION}-freenect-* ros-${ROS_VERSION}-vision-opencv ros-${ROS_VERSION}-rtabmap-ros ros-${ROS_VERSION}-scan-tools ros-${ROS_VERSION}-explore-lite libopencv-dev python-opencv ros-${ROS_VERSION}-rosbridge-server ros-${ROS_VERSION}-tf2-tools imagemagick fswebcam festival festvox-en1 libv4l-dev jq expect-dev curl libav-tools zbar-tools openssh-server libftdi1 libgif-dev pulseaudio pavucontrol ros-${ROS_VERSION}-pointcloud-to-laserscan git

if ! [[ ${TRAVIS} == "true" ]]; then
    sudo apt install -y ros-${ROS_VERSION}-turtlebot-apps ros-${ROS_VERSION}-turtlebot-interactions ros-${ROS_VERSION}-turtlebot-simulator
else
    printf "\n${GREEN}Skipping turtlebot bits forTravis CI Testing, because librealsense fails due to uvcvideo in Travis CI environment${NC}\n"
fi

# Update pip?
sudo -H pip install --upgrade pip
sudo chown -R ${USER} /home/${USER}/.cache/

# For 8-CH USB Relay board:
sudo -H pip install pylibftdi
# As of 4/27/2016 Rosbridge required me to install twisted via pip otherwise it failed.
sudo -H pip install twisted

if ! [[ -d ~/catkin_ws/src ]]; then
    printf "\n${YELLOW}[Creating the catkin workspace and testing with catkin_make]${NC}\n"
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/src
    catkin_init_workspace
    cd ~/catkin_ws/
    catkin_make
    source ~/catkin_ws/devel/setup.bash
    rospack profile
fi

# TODO: Replace ALL hector junk with:
# TODO: http://wiki.ros.org/explore_lite
printf "\n${YELLOW}[Cloning or Updating git repositories]${NC}\n"
#cd ~/catkin_ws/src
#if ! [[ -d ~/catkin_ws/src/hector_slam ]]
#    then
#    git clone https://github.com/tu-darmstadt-ros-pkg/hector_slam.git
#else
#    cd ~/catkin_ws/src/hector_slam
#    git pull
#fi
cd ~/catkin_ws/src

if ! [[ -d ~/catkin_ws/src/ArloBot ]]; then
    git clone -b new-serial-interface https://github.com/chrisl8/ArloBot.git
else
    cd ~/catkin_ws/src/ArloBot
    git pull
fi

#cd ~/catkin_ws/src
# If you have an XV-11 "Neato" Scanner
#if ! [[ -d ~/catkin_ws/src/xv_11_laser_driver ]]
#    then
#    git clone https://github.com/chrisl8/xv_11_laser_driver.git
#else
#    cd ~/catkin_ws/src/xv_11_laser_driver
#    git pull
#fi

# If you have a Scanse Sweep Scanner
if ! [[ -f /usr/local/lib/cmake/sweep/SweepConfig.cmake ]]; then
    cd
    git clone https://github.com/scanse/sweep-sdk.git
    cd ${HOME}/sweep-sdk/libsweep
    mkdir build
    cd build
    cmake .. -DCMAKE_BUILD_TYPE=Release
    cmake --build .
    sudo cmake --build . --target install
    sudo ldconfig
fi
cd ~/catkin_ws/src
if ! [[ -d ~/catkin_ws/src/sweep-ros ]]; then
    git clone https://github.com/scanse/sweep-ros.git
else
    cd ~/catkin_ws/src/sweep-ros
    git pull
fi
cd ~/catkin_ws/src
# If you have the excellent ROS by Example book now is a good time to clone the code for following along in the book:
if ! [[ -d ~/catkin_ws/src/rbx1 ]]; then
    git clone -b indigo-devel https://github.com/pirobot/rbx1.git
else
    cd ~/catkin_ws/src/rbx1
    git pull
fi
cd ~/catkin_ws/src
# If you want to use the USB Camera code from the ROS by Example book:
if ! [[ -d ~/catkin_ws/src/usb_cam ]]; then
    git clone https://github.com/bosch-ros-pkg/usb_cam.git
else
    cd ~/catkin_ws/src/usb_cam
    git pull
fi
cd ~/catkin_ws/src/ArloBot
if ! [[ ${TRAVIS} == "true" ]]; then
    if ! [[ -d ~/catkin_ws/src/ArloBot/mycroft-core ]]; then
        printf "\n${YELLOW}Do you want to install MyCroft on the Robot?${NC}\n"
        read -n 1 -s -r -p "Press 'y' if this is OK" RESPONSE_TO_MYCROFT_QUERY
        echo ""

        if [[ "${RESPONSE_TO_MYCROFT_QUERY}" == "y" ]]; then
            git clone -b master https://github.com/MycroftAI/mycroft-core.git
            cd ~/catkin_ws/src/ArloBot/mycroft-core
            ./dev_setup.sh
            ./start-mycroft.sh all
            printf "\n${YELLOW}Giving Mycoroft time to download skills.${NC}\n"
            #sleep 60
            #./stop-mycroft.sh
            #cd mycroft/tts/
            #ln -s ${HOME}/catkin_ws/src/ArloBot/mycroft-things/arlobot_tts.py

            printf "\n${YELLOW}[IF you want to use MyCroft:]${NC}\n"
            printf "\n${YELLOW}[Then see https://docs.mycroft.ai/development/cerberus for configuration info.]${NC}\n"
            printf "\n${YELLOW}[See more info at: https://docs.mycroft.ai/installing.and.running/installation/git.clone.install]${NC}\n"
            printf "\n${YELLOW}[At the least you will have to register MyCroft if you want full functionality, althoug it does work without registering.]${NC}\n"
        fi
    else
        cd ~/catkin_ws/src/ArloBot/mycroft-core
        ./stop-mycroft.sh
        git pull
        ./dev_setup.sh
        ./start-mycroft.sh all
    fi
    #cd ~/catkin_ws/src/ArloBot/mycroft-core
    #printf "\n${YELLO}Patching MyCroft TTS to include Arlobot TTS if we want it.{NC}\n"
    # git diff __init__.py > ~/catkin_ws/src/ArloBot/mycroft-things/tts_source_patch.diff
    #git apply ~/catkin_ws/src/ArloBot/mycroft-things/tts_source_patch.diff
else
    printf "\n${GREEN}Skipping Mycroft entirely for Travis CI Testing${NC}\n"
    # TODO: Could maybe test this, but ./dev_setup.sh asks interactive questions!
fi

if [[ -d /opt/mycroft/skills ]]; then
    if ! [[ -L /opt/mycroft/skills/arlobot-robot-skill ]]; then
        cd /opt/mycroft/skills/
        ln -s ${HOME}/catkin_ws/src/ArloBot/mycroft-arlobot-skill arlobot-robot-skill
    fi
    if ! [[ -L /opt/mycroft/skills/arlobot-smalltalk-skill ]]; then
        cd /opt/mycroft/skills/
        ln -s ${HOME}/catkin_ws/src/ArloBot/mycroft-smalltalk-skill arlobot-smalltalk-skill
    fi
fi

printf "\n${YELLOW}[(Re)Building ROS Source files.]${NC}\n"
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash
rospack profile

printf "\n${YELLOW}[Setting the ROS environment in your .bashrc file]${NC}\n"
if ! (grep ROS_HOSTNAME ~/.bashrc>/dev/null); then
    sh -c "echo \"export ROS_HOSTNAME=`uname -n`\" >> ~/.bashrc"
fi
if ! (grep ROSLAUNCH_SSH_UNKNOWN ~/.bashrc>/dev/null); then
    sh -c "echo \"export ROSLAUNCH_SSH_UNKNOWN=1\" >> ~/.bashrc"
fi
if ! (grep catkin_ws ~/.bashrc>/dev/null); then
    sh -c "echo \"source ~/catkin_ws/devel/setup.bash\" >> ~/.bashrc"
fi

printf "\n${YELLOW}[Setting up the Robot Package.]${NC}\n"

printf "\n${YELLOW}[Installing/Updating Node Version Manager]${NC}\n"
if [[ -e  ${HOME}/.nvm/nvm.sh ]]
    then
    printf "${YELLOW}Deactivating existing Node Version Manager:${NC}\n"
    export NVM_DIR="${HOME}/.nvm"
    [[ -s "$NVM_DIR/nvm.sh" ]] && . "$NVM_DIR/nvm.sh"  # This loads nvm
    nvm deactivate
fi

wget -qO- https://raw.githubusercontent.com/creationix/nvm/v0.34.0/install.sh | bash
export NVM_DIR="${HOME}/.nvm"
[[ -s "$NVM_DIR/nvm.sh" ]] && . "$NVM_DIR/nvm.sh"  # This loads nvm

printf "\n${YELLOW}[Initializing the Current Node LTS version]${NC}\n"
nvm install --lts
nvm alias default lts/*

printf "\n${YELLOW}[Updating npm]${NC}\n"
npm update -g npm

printf "\n${YELLOW}[Grabbing dependencies for node packages]${NC}\n"
printf "\n${YELLOW}You will get some errors here, that is normal. As long as things work, it is OK.$NC\n"
cd
if ! (which forever > /dev/null)
    then
    npm install -g forever
fi
if ! (which log.io-harvester > /dev/null)
    then
    npm install -g https://github.com/pruge/Log.io
fi
if ! (which pm2 > /dev/null)
    then
    npm install -g pm2
fi
cd ${HOME}/catkin_ws/src/ArloBot/node
printf "\n${YELLOW}You will get some errors here, that is normal. As long as things work, it is OK.$NC\n"
npm ci

cd ${HOME}/catkin_ws/src/ArloBot/website
npm ci
npm run build

if ! (which mjpg_streamer > /dev/null)
    then
    printf "\n${YELLOW}[Installing mjpg_streamer for Web Page camera viewing]${NC}\n"
    cd ${HOME}/catkin_ws/src/ArloBot/scripts
    svn co https://svn.code.sf.net/p/mjpg-streamer/code mjpg-streamer
    cd mjpg-streamer/mjpg-streamer
    # https://www.raspberrypi.org/forums/viewtopic.php?f=28&t=109352
    patch -p0 < ../../input_uvc_patch
    # sudo apt-get install libv4l-dev # Installed earlier, left here for documentation of why
    make USE_LIBV4L2=true clean all
    sudo make install
    # mjpg_streamer usage example:
    #mjpg_streamer -i "/usr/local/lib/input_uvc.so -d /dev/video0 -f 30 -r 640x480" -o "/usr/local/lib/output_http.so -p 58180 -w ${SCRIPT_DIR}/mjpg-streamer/mjpg-streamer/www"
fi

printf "\n${YELLOW}[Enable non-root use of Bluetooth 4.0.]${NC}\n"
sudo setcap cap_net_raw+eip $(eval readlink -f `which node`)

if ! [[ -f ${HOME}/Desktop/arlobot.desktop ]]
    then
    printf "\n${YELLOW}[Creating Desktop Icon]${NC}\n"
    if [[ ! -d ${HOME}/Desktop ]]; then
        mkdir ${HOME}/Desktop
    fi
    echo "[Desktop Entry]" > ${HOME}/Desktop/arlobot.desktop
    echo "Encoding=UTF-8" >> ${HOME}/Desktop/arlobot.desktop
    echo "Name=ArloBot" >> ${HOME}/Desktop/arlobot.desktop
    echo "GenericName=ArloBot" >> ${HOME}/Desktop/arlobot.desktop
    echo "Comment=Start the robot" >> ${HOME}/Desktop/arlobot.desktop
    if (which lxterminal)
        then
        echo "Exec=lxterminal --command \"${HOME}/catkin_ws/src/ArloBot/scripts/arlobotXwindows.sh\"" >> ${HOME}/Desktop/arlobot.desktop
    elif (which gnome-terminal)
        then
        echo "Exec=gnome-terminal --command \"${HOME}/catkin_ws/src/ArloBot/scripts/arlobotXwindows.sh\"" >> ${HOME}/Desktop/arlobot.desktop
    fi
    echo "Icon=${HOME}/catkin_ws/src/ArloBot/icon-70x70.png" >> ${HOME}/Desktop/arlobot.desktop
    echo "Type=Application" >> ${HOME}/Desktop/arlobot.desktop
    echo "Path=${HOME}/catkin_ws/src/ArloBot/scripts/" >> ${HOME}/Desktop/arlobot.desktop
    echo "Terminal=false" >> ${HOME}/Desktop/arlobot.desktop
    chmod +x ${HOME}/Desktop/arlobot.desktop
fi

if [[ ! -d ${HOME}/.arlobot ]]
    then
    printf "\n${YELLOW}[Setting up .arlobot folder]${NC}\n"
    printf "${GREEN}This holds personal data for your robot.${NC}\n"
    # We will use ~/.arlobot to store "private" data
    # That is data that doesn't need to be part of
    # the public github repo like user tokens,
    # sounds, and room maps and per robot settings
    mkdir ${HOME}/.arlobot
fi

ARLO_HOME=${HOME}/.arlobot

if [[ -e ${ARLO_HOME}/personalDataForBehavior.json ]]
    then
    node ${HOME}/catkin_ws/src/ArloBot/node/personalData.js
else
    printf "\n"
    cp ${HOME}/catkin_ws/src/ArloBot/scripts/dotarlobot/personalDataForBehavior.json ${ARLO_HOME}/
    printf "${GREEN}A brand new ${RED}~/.arlobot/personalDataForBehavior.json${GREEN} file has been created,${NC}\n"
    printf "${LIGHT_PURPLE}Please edit this file to customize according to your robot!${NC}\n"
fi

if [[ ! -d ${ARLO_HOME}/rosmaps ]]
    then
    mkdir ${ARLO_HOME}/rosmaps
fi

if [[ ! -d ${ARLO_HOME}/sounds ]]
    then
    mkdir ${ARLO_HOME}/sounds
fi

if [[ ! -d ${ARLO_HOME}/status ]]
    then
    mkdir ${ARLO_HOME}/status
fi
sudo chmod -R 777 ${ARLO_HOME}/status

if [[ ! -d ${ARLO_HOME}/status/doors ]]
    then
    mkdir ${ARLO_HOME}/status/doors
fi
sudo chmod -R 777 ${ARLO_HOME}/status/doors

if ! [[ -f /etc/udev/rules.d/99-libftdi.rules ]]
    then
    printf "\n${YELLOW}[Adding required sudo rule to reset USB ports.]${NC}\n"
    sudo ${HOME}/catkin_ws/src/ArloBot/scripts/addRuleForUSBRelayBoard.sh
    printf "${RED}You may have to reboot before the USB Relay board will function!${NC}\n"
fi

if ! (grep mbrola /etc/festival.scm>/dev/null)
    then
    sudo ${HOME}/catkin_ws/src/ArloBot/scripts/updateFestivalDefaults.sh
fi

if ! (sudo -nl|grep resetUSB > /dev/null)
    then
    printf "\n${YELLOW}[Setting up required sudo entries.]${NC}\n"
    echo "${USER} ALL = NOPASSWD: ${HOME}/catkin_ws/src/ArloBot/scripts/resetUSB.sh" >> /tmp/arlobot_sudoers
    chmod 0440 /tmp/arlobot_sudoers
    sudo chown root:root /tmp/arlobot_sudoers
    sudo mv /tmp/arlobot_sudoers /etc/sudoers.d/
    sudo chown root:root /etc/sudoers.d/arlobot_sudoers
fi

if [[ "${USER}" == chrisl8 ]]
    then
    if ! [[ -d /home/robotStatusUser ]]
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
    # For use with the paid text to speech engine I use
    #if ! (which aoss > /dev/null)
    #    then
    #    printf "\n${YELLOW}[Adding aoss for Text To Speech.]${NC}\n"
    #    sudo apt install -y alsa-oss
    #    printf "\n${GREEN}Don't for get to insatll Cepstral Voice!${NC}\n"
    #fi

    # Special notices for the developer himself to keep his stuff up to date!
    cd ${HOME}/catkin_ws/src/ArloBot/node
    printf "\n${RED}[Hey ${USER} please make sure the below items are up to date!]${NC}\n"
    printf "\n${GREEN}[Hey ${USER} please make sure the below items are up to date!]${NC}\n"
    printf "\n${PURPLE}[Hey ${USER} please make sure the below items are up to date!]${NC}\n"
    printf "${YELLOW}Does the current version of nvm we installed:${NC} "
    nvm --version
    printf "${YELLOW}Match the version on github:${NC} "
    wget -qO- https://github.com/creationix/nvm/blob/master/README.md|grep install.sh|grep wget|sed -e "s/<pre><code>//"|sed "s/\//\\n/g"|grep ^v|head -1
    printf "\n${YELLOW}You are using this version of node:${NC} "
    node --version
    printf "${YELLOW}and this is the current stable version of node:${NC} "
    wget -qO- https://nodejs.org/en/download/|grep "Latest LTS Version:"|sed "s/<\/p>//g"|sed "s/<p class=\"color-lightgray\">//"
    printf "\n${YELLOW}Checking for out of date global node modules:${NC}\n"
    npm outdated -g
    printf "${YELLOW}Checking for out of date package node modules:${NC}\n"
    printf "${YELLOW}in /node:${NC}\n"
    npm outdated
    printf "${YELLOW}in /website:${NC}\n"
    cd ${HOME}/catkin_ws/src/ArloBot/website
    npm outdated
    printf "${PURPLE}-------------------------------------------------------${NC}\n"
fi

# Arlobot Specific settings:

if ! (id|grep dialout>/dev/null); then
    printf "\n${GREEN}Adding your user to the 'dialout' group.${NC}\n"
    sudo adduser ${USER} dialout > /dev/null
    printf "${RED}You may have to reboot before you can use the Propeller Board.${NC}\n"
fi

if ! (id|grep video>/dev/null); then
    printf "\n${GREEN}Adding your user to the 'video' group for access to cameras.${NC}\n"
    sudo adduser ${USER} video > /dev/null
fi

if ! (which simpleide > /dev/null); then
    printf "\n${YELLOW}[Setting up Parallax SimpleIDE for putting code on Activity Board.]${NC}\n"
    cd /tmp
    wget https://web.archive.org/web/20161005174013/http://downloads.parallax.com/plx/software/side/101rc1/simple-ide_1-0-1-rc1_amd64.deb
    sudo dpkg -i /tmp/simple-ide_1-0-1-rc1_amd64.deb
    rm /tmp/simple-ide_1-0-1-rc1_amd64.deb
fi

if ! [[ -e ~/Documents/SimpleIDE/Learn/Simple\ Libraries/Robotics/Arlo/libarlodrive/arlodrive.c ]]; then
    printf "\n${YELLOW}[You must Update your SimpleIDE Learn Folder using the instructions here!]${NC}\n"
    printf "\n${GREEN}http://learn.parallax.com/tutorials/language/propeller-c/propeller-c-set-simpleide/update-your-learn-folder${NC}\n"
fi

# We will use ~/.arlobot to store "private" data
# That is data that doesn't need to be part of
# the public github repo like user tokens,
# sounds, and room maps and per robot settings
if ! [[ -d ${HOME}/.arlobot ]]; then
    mkdir ${HOME}/.arlobot
fi

ARLO_HOME=${HOME}/.arlobot

if [[ -e ${ARLO_HOME}/arlobot.yaml ]]; then
    if ! (diff ${HOME}/catkin_ws/src/ArloBot/src/arlobot/arlobot_bringup/param/arlobot.yaml ${ARLO_HOME}/arlobot.yaml > /dev/null); then
        printf "\n${GREEN}The arlobot.yaml file in the repository is different from the one${NC}\n"
        printf "${GREEN}in your local settings.${NC}\n"
        printf "${GREEN}This is expected, but just in case, please look over the differences,${NC}\n"
        printf "${GREEN}and see if you need to copy in any new settings, or overwrite the file completely:${NC}\n"
        diff ${HOME}/catkin_ws/src/ArloBot/src/arlobot/arlobot_bringup/param/arlobot.yaml ${ARLO_HOME}/arlobot.yaml
        cp -i ${HOME}/catkin_ws/src/ArloBot/src/arlobot/arlobot_bringup/param/arlobot.yaml ${ARLO_HOME}/
        printf "\n"
    fi
else
    printf "\n"
    cp ${HOME}/catkin_ws/src/ArloBot/src/arlobot/arlobot_bringup/param/arlobot.yaml ${ARLO_HOME}/
    printf "${GREEN}A brand new ${RED}~/.arlobot/arlobot.yaml${GREEN} file has been created,${NC}\n"
    printf "${LIGHT_PURPLE}Please edit this file to customize according to your robot!${NC}\n"
fi

for i in `ls ${HOME}/catkin_ws/src/ArloBot/PropellerCodeForArloBot/dotfiles/`
do
    if [[ -e  ${ARLO_HOME}/${i} ]]; then
        if ! (diff ${HOME}/catkin_ws/src/ArloBot/PropellerCodeForArloBot/dotfiles/${i} ${ARLO_HOME}/${i} > /dev/null); then
            printf "\n${GREEN}The ${RED}${i}${GREEN} file in the repository is different from the one${NC}\n"
            printf "${GREEN}in your local settings.${NC}\n"
            printf "${GREEN}This is expected, but just in case, please look over the differences,${NC}\n"
            printf "${GREEN}and see if you need to copy in any new settings, or overwrite the file completely:${NC}\n"
            diff ${HOME}/catkin_ws/src/ArloBot/PropellerCodeForArloBot/dotfiles/${i} ${ARLO_HOME}/${i}
            cp -i ${HOME}/catkin_ws/src/ArloBot/PropellerCodeForArloBot/dotfiles/${i} ${ARLO_HOME}/
        fi
    else
        printf "\n"
        cp ${HOME}/catkin_ws/src/ArloBot/PropellerCodeForArloBot/dotfiles/${i} ${ARLO_HOME}/
        printf "${GREEN}A brand new ${RED}${ARLO_HOME}/${i}${GREEN} file has been created,${NC}\n"
        printf "${LIGHT_PURPLE}Please edit this file to customize according to your robot!${NC}\n"
    fi
done

printf "\n${PURPLE}Anytime you want to update ArloBot code from the web you can run this same script again. It will pull down and compile new code without wiping out custom configs in ~/.arlarbot. I run this script myself almost every day.\n"

printf "\n${YELLOW}-----------------------------------${NC}\n"
printf "${YELLOW}ALL DONE! REBOOT AND START TESTING!${NC}\n"
printf "${BLUE}I have a list of tests here: cat ${HOME}/catkin_ws/src/ArloBot/manualTests.txt${NC}\n"
printf "${GREEN}Look at README.md for testing ideas.${NC}\n"
printf "${GREEN}See here for your next step: ${BLUE}http://ekpyroticfrood.net/?p=165\n${NC}\n"
