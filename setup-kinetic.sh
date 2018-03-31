#!/bin/bash
# ROS Indigo Automated Install - credit goes to everyone
# Blame goes to ChrisL8

# Run this straight off of github like this:
# bash <(wget -qO- --no-cache https://raw.githubusercontent.com/chrisl8/ArloBot/master/setup.sh)

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

printf "\n${YELLOW}SETTING UP ROBOT OPERATING SYSTEM FOR YOUR ARLOBOT!${NC}\n"
printf "${YELLOW}---------------------------------------------------${NC}\n"
printf "${GREEN}You will be asked for your password for running commands as root!${NC}\n"

version=`lsb_release -sc`

printf "\n${YELLOW}[Checking the Ubuntu version]${NC}\n"
printf "${BLUE}Ubuntu ${version} found${NC}\n"
case $version in
  "xenial")
;;
*)
printf "${RED}[This script will only work on ubuntu xenial(16.04)]${NC}\n"
exit 1
esac

# I never use this, but if you are having time issues maybe uncomment this.
#printf "${YELLOW}[Installing chrony and setting the ntpdate]${NC}\n"
#sudo apt-get install -y chrony
#sudo ntpdate ntp.ubuntu.com

if ! [ -e /etc/apt/sources.list.d/ros-latest.list ]
    then
    printf "${YELLOW}[Adding the ROS repository]${NC}\n"
    # This should follow the official ROS install instructions closely.
    # That is why there is a separate section for extra packages that I need for Arlo.
    sudo sh -c "echo \"deb http://packages.ros.org/ros/ubuntu ${version} main\" > /etc/apt/sources.list.d/ros-latest.list"
    printf "${BLUE}[Checking the ROS keys]${NC}\n"
    roskey=`apt-key list | grep -i "ROS builder"`
    if [ -z "$roskey" ]
        then
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
if ! (dpkg -s ros-kinetic-desktop-full|grep "Status: install ok installed" &> /dev/null)
    then
    printf "\n${YELLOW}[Installing ROS]${NC}\n"
    sudo apt install -y ros-kinetic-desktop-full
    printf "${YELLOW}[ROS installed!]${NC}\n"
    printf "\n${YELLOW}[rosdep init and python-rosinstall]${NC}\n"
    if ! [ -e /etc/ros/rosdep/sources.list.d/20-default.list ]
        then
        sudo sh -c "rosdep init"
    fi
    printf "${BLUE}Running rosdep update . . .${NC}\n"
    rosdep update
    source /opt/ros/kinetic/setup.bash
    printf "${BLUE}Installing python-rosinstall:${NC}\n"
    sudo apt install -y python-rosinstall
    # END Offical ROS Install section
fi

# In case .bashrc wasn't set up, or you didn't reboot
if ! (which catkin_make > /dev/null)
    then
    source /opt/ros/kinetic/setup.bash
fi

printf "\n${YELLOW}[Installing additional Ubuntu and ROS Packages for Arlo]${NC}\n"
printf "${BLUE}This runs every time, in case new packages were added.${NC}\n"
# Notes on what the packages are for:

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
#ros-kinetic-pointcloud-to-laserscan for Scanse Sweep
#ros-kinetic-geodesy for hector compile, might not need it if I stop using hector_explore
#libceres-dev for hector compile, might not need it if I stop using hector_explore

sudo apt install -y ros-kinetic-rqt-* ros-kinetic-turtlebot ros-kinetic-turtlebot-apps ros-kinetic-turtlebot-interactions ros-kinetic-turtlebot-simulator ros-kinetic-kobuki-ftdi python-ftdi1 python-pip python-serial ros-kinetic-openni-* ros-kinetic-openni2-* ros-kinetic-freenect-* ros-kinetic-vision-opencv ros-kinetic-rtabmap-ros ros-kinetic-scan-tools ros-kinetic-explore-lite libopencv-dev python-opencv ros-kinetic-rosbridge-server imagemagick fswebcam festival festvox-en1 libv4l-dev jq expect-dev curl libav-tools zbar-tools openssh-server libftdi1 libgif-dev pulseaudio pavucontrol ros-kinetic-pointcloud-to-laserscan

# Update pip?
pip install --upgrade pip

# For 8-CH USB Relay board:
sudo -H pip install pylibftdi
# As of 4/27/2016 Rosbridge required me to install twisted via pip otherwise it failed.
sudo -H pip install twisted

if ! [ -d ~/catkin_ws/src ]
    then
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
#if ! [ -d ~/catkin_ws/src/hector_slam ]
#    then
#    git clone https://github.com/tu-darmstadt-ros-pkg/hector_slam.git
#else
#    cd ~/catkin_ws/src/hector_slam
#    git pull
#fi
cd ~/catkin_ws/src
if ! [ -d ~/catkin_ws/src/ArloBot ]
    then
    git clone -b kinetic https://github.com/chrisl8/ArloBot.git
else
    cd ~/catkin_ws/src/ArloBot
    git pull
fi
#cd ~/catkin_ws/src
# If you have an XV-11 "Neato" Scanner
#if ! [ -d ~/catkin_ws/src/xv_11_laser_driver ]
#    then
#    git clone https://github.com/chrisl8/xv_11_laser_driver.git
#else
#    cd ~/catkin_ws/src/xv_11_laser_driver
#    git pull
#fi
cd ~/catkin_ws/src
# If you have a Scanse Sweep Scanner
if ! [ -d ~/catkin_ws/src/sweep-ros ]
    then
    git clone https://github.com/scanse/sweep-ros.git
else
    cd ~/catkin_ws/src/sweep-ros
    git pull
fi
cd ~/catkin_ws/src
# If you have the excellent ROS by Example book now is a good time to clone the code for following along in the book:
if ! [ -d ~/catkin_ws/src/rbx1 ]
    then
    git clone -b indigo-devel https://github.com/pirobot/rbx1.git
else
    cd ~/catkin_ws/src/rbx1
    git pull
fi
cd ~/catkin_ws/src
# If you want to use the USB Camera code from the ROS by Example book:
if ! [ -d ~/catkin_ws/src/usb_cam ]
    then
    git clone https://github.com/bosch-ros-pkg/usb_cam.git
else
    cd ~/catkin_ws/src/usb_cam
    git pull
fi
cd ~/catkin_ws/src/ArloBot
if ! [ -d ~/catkin_ws/src/ArloBot/mycroft-core ]
then
    printf "\n${YELLOW}Do you want to install MyCroft on the Robot?${NC}\n"
    read -n 1 -s -r -p "Press 'y' if this is OK" RESPONSE_TO_MYCROFT_QUERY
    echo ""

    if [ "${RESPONSE_TO_MYCROFT_QUERY}" == "y" ]
    then
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
    ./start-mycroft.sh all
fi
    #cd ~/catkin_ws/src/ArloBot/mycroft-core
    #printf "\n${YELLO}Patching MyCroft TTS to include Arlobot TTS if we want it.{NC}\n"
    # git diff __init__.py > ~/catkin_ws/src/ArloBot/mycroft-things/tts_source_patch.diff
    #git apply ~/catkin_ws/src/ArloBot/mycroft-things/tts_source_patch.diff

if [ -d /opt/mycroft/skills ]
then
    if ! [ -L /opt/mycroft/skills/arlobot-robot-skill ]; then
        cd /opt/mycroft/skills/
        ln -s ${HOME}/catkin_ws/src/ArloBot/mycroft-arlobot-skill arlobot-robot-skill
    fi
    if ! [ -L /opt/mycroft/skills/arlobot-smalltalk-skill ]; then
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
if ! (grep ROS_HOSTNAME ~/.bashrc>/dev/null)
    then
    sh -c "echo \"export ROS_HOSTNAME=`uname -n`\" >> ~/.bashrc"
fi
if ! (grep ROSLAUNCH_SSH_UNKNOWN ~/.bashrc>/dev/null)
    then
    sh -c "echo \"export ROSLAUNCH_SSH_UNKNOWN=1\" >> ~/.bashrc"
fi
if ! (grep catkin_ws ~/.bashrc>/dev/null)
    then
    sh -c "echo \"source ~/catkin_ws/devel/setup.bash\" >> ~/.bashrc"
fi

printf "\n${YELLOW}[Setting up the Robot Package.]${NC}\n"
# Run Robot Setup Script:
~/catkin_ws/src/ArloBot/scripts/setup.sh

# Arlobot Specific settings:

if ! (id|grep dialout>/dev/null)
    then
    printf "\n${GREEN}Adding your user to the 'dialout' group.${NC}\n"
    sudo adduser ${USER} dialout > /dev/null
    printf "${RED}You may have to reboot before you can use the Propeller Board.${NC}\n"
fi

if ! (id|grep video>/dev/null)
    then
    printf "\n${GREEN}Adding your user to the 'video' group for access to cameras.${NC}\n"
    sudo adduser ${USER} video > /dev/null
fi

if ! (which simpleide > /dev/null)
then
    printf "\n${YELLOW}[Setting up Parallax SimpleIDE for putting code on Activity Board.]${NC}\n"
    cd /tmp
    wget https://web.archive.org/web/20161005174013/http://downloads.parallax.com/plx/software/side/101rc1/simple-ide_1-0-1-rc1_amd64.deb
    sudo dpkg -i /tmp/simple-ide_1-0-1-rc1_amd64.deb
    rm /tmp/simple-ide_1-0-1-rc1_amd64.deb
fi

if ! [ -e ~/Documents/SimpleIDE/Learn/Simple\ Libraries/Robotics/Arlo/libarlodrive/arlodrive.c ]
then
    printf "\n${YELLOW}[You must Update your SimpleIDE Learn Folder using the instructions here!]${NC}\n"
    print "\n${GREEN}http://learn.parallax.com/tutorials/language/propeller-c/propeller-c-set-simpleide/update-your-learn-folder${NC}\n"
fi

# We will use ~/.arlobot to store "private" data
# That is data that doesn't need to be part of
# the public github repo like user tokens,
# sounds, and room maps and per robot settings
if ! [ -d ${HOME}/.arlobot ]
    then
    mkdir ${HOME}/.arlobot
fi

ARLOHOME=${HOME}/.arlobot

if [ -e ${ARLOHOME}/arlobot.yaml ]
    then
    if ! (diff ${HOME}/catkin_ws/src/ArloBot/src/arlobot/arlobot_bringup/param/arlobot.yaml ${ARLOHOME}/arlobot.yaml > /dev/null)
        then
        printf "\n${GREEN}The arlobot.yaml file in the repository is different from the one${NC}\n"
        printf "${GREEN}in your local settings.${NC}\n"
        printf "${GREEN}This is expected, but just in case, please look over the differences,${NC}\n"
        printf "${GREEN}and see if you need to copy in any new settings, or overwrite the file completely:${NC}\n"
        diff ${HOME}/catkin_ws/src/ArloBot/src/arlobot/arlobot_bringup/param/arlobot.yaml ${ARLOHOME}/arlobot.yaml
        cp -i ${HOME}/catkin_ws/src/ArloBot/src/arlobot/arlobot_bringup/param/arlobot.yaml ${ARLOHOME}/
        printf "\n"
    fi
else
    printf "\n"
    cp ${HOME}/catkin_ws/src/ArloBot/src/arlobot/arlobot_bringup/param/arlobot.yaml ${ARLOHOME}/
    printf "${GREEN}A brand new ${RED}~/.arlobot/arlobot.yaml${GREEN} file has been created,${NC}\n"
    printf "${LIGHTPURPLE}Please edit this file to customize according to your robot!${NC}\n"
fi

for i in `ls ${HOME}/catkin_ws/src/ArloBot/PropellerCodeForArloBot/dotfiles/`
do
    if [ -e  ${ARLOHOME}/${i} ]
        then
        if ! (diff ${HOME}/catkin_ws/src/ArloBot/PropellerCodeForArloBot/dotfiles/${i} ${ARLOHOME}/${i} > /dev/null)
            then
            printf "\n${GREEN}The ${RED}${i}${GREEN} file in the repository is different from the one${NC}\n"
            printf "${GREEN}in your local settings.${NC}\n"
            printf "${GREEN}This is expected, but just in case, please look over the differences,${NC}\n"
            printf "${GREEN}and see if you need to copy in any new settings, or overwrite the file completely:${NC}\n"
            diff ${HOME}/catkin_ws/src/ArloBot/PropellerCodeForArloBot/dotfiles/${i} ${ARLOHOME}/${i}
            cp -i ${HOME}/catkin_ws/src/ArloBot/PropellerCodeForArloBot/dotfiles/${i} ${ARLOHOME}/
        fi
    else
        printf "\n"
        cp ${HOME}/catkin_ws/src/ArloBot/PropellerCodeForArloBot/dotfiles/${i} ${ARLOHOME}/
        printf "${GREEN}A brand new ${RED}${ARLOHOME}/${i}${GREEN} file has been created,${NC}\n"
        printf "${LIGHTPURPLE}Please edit this file to customize according to your robot!${NC}\n"
    fi
done

printf "\n${PURPLE}Anytime you want to update ArloBot code from the web you can run this same script again. It will pull down and compile new code without wiping out custom configs in ~/.arlarbot. I run this script myself almost every day.\n"

printf "\n${YELLOW}-----------------------------------${NC}\n"
printf "${YELLOW}ALL DONE! REBOOT AND START TESTING!${NC}\n"
printf "${BLUE}I have a list of tests here: cat ${HOME}/catkin_ws/src/ArloBot/manualTests.txt${NC}\n"
printf "${GREEN}Look at README.md for testing ideas.${NC}\n"
printf "${GREEN}See here for your next step: ${BLUE}http://ekpyroticfrood.net/?p=165\n${NC}\n"
