#!/bin/bash
# ROS Lunar "Workstation" Automated Install - credit goes to everyone
# Blame goes to ChrisL8
# This is to set up enough of ROS to use RVIZ and some other GUI tools,
# on a secondary system. It will not run a robot.

# Run this straight off of github like this:
# bash <(wget -qO- --no-cache https://raw.githubusercontent.com/chrisl8/ArloBot/kinetic/workstation-lunar.sh)

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

printf "\n${YELLOW}SETTING UP ROS LUNAR FOR YOUR REMOTE WORK!${NC}\n"
printf "${YELLOW}-------------------------------------------${NC}\n"
printf "${GREEN}You will be asked for your password for running commands as root!${NC}\n"

version=`lsb_release -sc`

printf "\n${YELLOW}[Checking the Ubuntu version]${NC}\n"
printf "${BLUE}Ubuntu ${version} found${NC}\n"
case ${version} in
  "yakkety" | "xenial" | "zesty")
;;
*)
    printf "${RED}[This script will only work on Ubuntu Zesty (17.04), Yakkety (16.10), and Xenial (16.04)]${NC}\n"
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
if ! (dpkg -s ros-lunar-desktop|grep "Status: install ok installed" &> /dev/null)
    then
    printf "\n${YELLOW}[Installing ROS]${NC}\n"
    sudo apt install -y ros-lunar-desktop
    printf "${YELLOW}[ROS installed!]${NC}\n"
    printf "\n${YELLOW}[rosdep init and python-rosinstall]${NC}\n"
    if ! [ -e /etc/ros/rosdep/sources.list.d/20-default.list ]
        then
        sudo sh -c "rosdep init"
    fi
    printf "${BLUE}Running rosdep update . . .${NC}\n"
    rosdep update
    source /opt/ros/lunar/setup.bash
    printf "${BLUE}Installing python-rosinstall:${NC}\n"
    sudo apt install -y python-rosinstall
    # END Offical ROS Install section
fi

# In case .bashrc wasn't set up, or you didn't reboot
if ! (which catkin_make > /dev/null)
    then
    source /opt/ros/lunar/setup.bash
fi

printf "\n${YELLOW}[Installing additional Ubuntu and ROS Packages for Arlo]${NC}\n"
printf "${BLUE}This runs every time, in case new packages were added.${NC}\n"
# Notes on what the packages are for:
# vlc-nox is for listening to rebot's microphone.
# zbar-tools python-qrtools qtqr for generating and reading QR Codes.
# rtabmap is for 3D mapping
# jq allows shell scripts to read .json formatted config files.

sudo apt install -y vlc-nox zbar-tools python-qrtools qtqr ros-lunar-rtabmap-ros jq

if ! [ -d ~/catkin_ws/src ]
    then
    printf "\n${YELLOW}[Creating the catkin workspace and testing with catkin_make]${NC}\n"
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/src
    catkin_init_workspace
    cd ~/catkin_ws/
    catkin_make
    source ${HOME}/catkin_ws/devel/setup.bash
    rospack profile
fi

printf "\n${YELLOW}[Cloning or Updating git repositories]${NC}\n"
cd ~/catkin_ws/src
if ! [ -d ~/catkin_ws/src/hector_slam ]
    then
    git clone https://github.com/chrisl8/hector_slam.git
else
    cd ~/catkin_ws/src/hector_slam
    git pull
fi
cd ~/catkin_ws/src
if ! [ -d ~/catkin_ws/src/hector_navigation ]
    then
    git clone https://github.com/chrisl8/hector_navigation.git
else
    cd ~/catkin_ws/src/hector_navigation
    git pull
fi
cd ~/catkin_ws/src
if ! [ -d ~/catkin_ws/src/ArloBot ]
    then
    git clone -b kinetic https://github.com/chrisl8/ArloBot.git
else
    cd ~/catkin_ws/src/ArloBot
    git pull
fi
cd ~/catkin_ws/src
# If you have an XV-11 "Neato" Scanner
if ! [ -d ~/catkin_ws/src/xv_11_laser_driver ]
    then
    git clone https://github.com/chrisl8/xv_11_laser_driver.git
else
    cd ~/catkin_ws/src/xv_11_laser_driver
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
cd ~/catkin_ws/src

if ! [ -f ${HOME}/Desktop/listen2robot.desktop ]
    then
    printf "\n${YELLOW}[Creating Desktop Icon to Listen to Robot's Microphone]${NC}\n"
    echo "[Desktop Entry]" > ${HOME}/Desktop/listen2robot.desktop
    echo "Encoding=UTF-8" >> ${HOME}/Desktop/listen2robot.desktop
    echo "Name=Listen 2 Robot" >> ${HOME}/Desktop/listen2robot.desktop
    echo "GenericName=listen2robot" >> ${HOME}/Desktop/listen2robot.desktop
    echo "Comment=Listen to robot" >> ${HOME}/Desktop/listen2robot.desktop
    if (which lxterminal > /dev/null)
        then
        echo "Exec=lxterminal --command \"${HOME}/catkin_ws/src/ArloBot/scripts/listenToArlobot.sh\"" >> ${HOME}/Desktop/listen2robot.desktop
    elif (which gnome-terminal > /dev/null)
        then
        echo "Exec=gnome-terminal --command \"${HOME}/catkin_ws/src/ArloBot/scripts/listenToArlobot.sh\"" >> ${HOME}/Desktop/listen2robot.desktop
    fi
    echo "Icon=${HOME}/catkin_ws/src/ArloBot/icon-70x70.png" >> ${HOME}/Desktop/listen2robot.desktop
    echo "Type=Application" >> ${HOME}/Desktop/listen2robot.desktop
    echo "Path=${HOME}/catkin_ws/src/ArloBot/scripts/" >> ${HOME}/Desktop/listen2robot.desktop
    echo "Terminal=false" >> ${HOME}/Desktop/listen2robot.desktop
    chmod +x ${HOME}/Desktop/listen2robot.desktop
fi

if ! [ -f ${HOME}/.local/share/applications/listen2robot.desktop ]
    then
    printf "\n${YELLOW}[Creating GNOME Icon to Listen to Robot's Microphone]${NC}\n"
    echo "[Desktop Entry]" > ${HOME}/.local/share/applications/listen2robot.desktop
    echo "Encoding=UTF-8" >> ${HOME}/.local/share/applications/listen2robot.desktop
    echo "Name=Listen 2 Robot" >> ${HOME}/.local/share/applications/listen2robot.desktop
    echo "GenericName=listen2robot" >> ${HOME}/.local/share/applications/listen2robot.desktop
    echo "Comment=Listen to robot" >> ${HOME}/.local/share/applications/listen2robot.desktop
    if (which lxterminal > /dev/null)
        then
        echo "Exec=lxterminal --command \"${HOME}/catkin_ws/src/ArloBot/scripts/listenToArlobot.sh\"" >> ${HOME}/.local/share/applications/listen2robot.desktop
    elif (which gnome-terminal > /dev/null)
        then
        echo "Exec=gnome-terminal --command \"${HOME}/catkin_ws/src/ArloBot/scripts/listenToArlobot.sh\"" >> ${HOME}/.local/share/applications/listen2robot.desktop
    fi
    echo "Icon=${HOME}/catkin_ws/src/ArloBot/icon-70x70.png" >> ${HOME}/.local/share/applications/listen2robot.desktop
    echo "Type=Application" >> ${HOME}/.local/share/applications/listen2robot.desktop
    echo "Path=${HOME}/catkin_ws/src/ArloBot/scripts/" >> ${HOME}/.local/share/applications/listen2robot.desktop
    echo "Terminal=false" >> ${HOME}/.local/share/applications/listen2robot.desktop
    chmod +x ${HOME}/.local/share/applications/listen2robot.desktop
fi

if ! [ -f ${HOME}/Desktop/RVIZ.desktop ]
    then
    printf "\n${YELLOW}[Creating Desktop Icon to run RVIZ]${NC}\n"
    echo "[Desktop Entry]" > ${HOME}/Desktop/RVIZ.desktop
    echo "Encoding=UTF-8" >> ${HOME}/Desktop/RVIZ.desktop
    echo "Name=RVIZ" >> ${HOME}/Desktop/RVIZ.desktop
    echo "GenericName=RVIZ" >> ${HOME}/Desktop/RVIZ.desktop
    echo "Comment=RVIZ" >> ${HOME}/Desktop/RVIZ.desktop
    if (which lxterminal > /dev/null)
        then
        echo "Exec=lxterminal --command \"bash -ci ${HOME}/catkin_ws/src/ArloBot/scripts/view-navigation.sh\"" >> ${HOME}/Desktop/RVIZ.desktop
    elif (which gnome-terminal > /dev/null)
        then
        echo "Exec=gnome-terminal --command \"bash -ci ${HOME}/catkin_ws/src/ArloBot/scripts/view-navigation.sh\"" >> ${HOME}/Desktop/RVIZ.desktop
    fi
    echo "Icon=${HOME}/catkin_ws/src/ArloBot/icon-70x70.png" >> ${HOME}/Desktop/RVIZ.desktop
    echo "Type=Application" >> ${HOME}/Desktop/RVIZ.desktop
    echo "Path=${HOME}/catkin_ws/src/ArloBot/scripts/" >> ${HOME}/Desktop/RVIZ.desktop
    echo "Terminal=false" >> ${HOME}/Desktop/RVIZ.desktop
    chmod +x ${HOME}/Desktop/RVIZ.desktop
fi

if ! [ -f ${HOME}/.local/share/applications/RVIZ.desktop ]
    then
    printf "\n${YELLOW}[Creating GNOME Icon to run RVIZ]${NC}\n"
    echo "[Desktop Entry]" > ${HOME}/.local/share/applications/RVIZ.desktop
    echo "Encoding=UTF-8" >> ${HOME}/.local/share/applications/RVIZ.desktop
    echo "Name=RVIZ" >> ${HOME}/.local/share/applications/RVIZ.desktop
    echo "GenericName=RVIZ" >> ${HOME}/.local/share/applications/RVIZ.desktop
    echo "Comment=RVIZ" >> ${HOME}/.local/share/applications/RVIZ.desktop
    if (which lxterminal > /dev/null)
        then
        echo "Exec=lxterminal --command \"bash -ci ${HOME}/catkin_ws/src/ArloBot/scripts/view-navigation.sh\"" >> ${HOME}/.local/share/applications/RVIZ.desktop
    elif (which gnome-terminal > /dev/null)
        then
        echo "Exec=gnome-terminal --command \"bash -ci ${HOME}/catkin_ws/src/ArloBot/scripts/view-navigation.sh\"" >> ${HOME}/.local/share/applications/RVIZ.desktop
    fi
    echo "Icon=${HOME}/catkin_ws/src/ArloBot/icon-70x70.png" >> ${HOME}/.local/share/applications/RVIZ.desktop
    echo "Type=Application" >> ${HOME}/.local/share/applications/RVIZ.desktop
    echo "Path=${HOME}/catkin_ws/src/ArloBot/scripts/" >> ${HOME}/.local/share/applications/RVIZ.desktop
    echo "Terminal=false" >> ${HOME}/.local/share/applications/RVIZ.desktop
    chmod +x ${HOME}/.local/share/applications/RVIZ.desktop
fi

printf "\n${YELLOW}[NOT Building ROS Source files.]${NC}\n"
printf "${BLUE}The ArloBot source will not build in Lunar yet.${NC}\n"
printf "${BLUE}The unbuilt files are enough to allow RVIZ and other GUI tools to work.${NC}\n"
source ${HOME}/catkin_ws/devel/setup.bash

# We will use ~/.arlobot to store "private" data
# That is data that doesn't need to be part of
# the public github repo like user tokens,
# sounds, and room maps and per robot settings
if ! [ -d ${HOME}/.arlobot ]
    then
    mkdir ${HOME}/.arlobot
fi

ARLOHOME=${HOME}/.arlobot

if [ -e ${ARLOHOME}/personalDataForBehavior.json ]
    then
    node ~/catkin_ws/src/ArloBot/node/personalData.js
else
    printf "\n"
    cp ~/catkin_ws/src/ArloBot/scripts/dotarlobot/personalDataForBehavior.json ${ARLOHOME}/
    printf "${GREEN}A brand new ${RED}~/.arlobot/personalDataForBehavior.json${GREEN} file has been created,${NC}\n"
    printf "${LIGHTPURPLE}Please edit this file to customize according to your robot!${NC}\n"
fi

printf "\n${YELLOW}[Setting the ROS environment in your .bashrc file]${NC}\n"
if ! (grep ROS_MASTER_URI ~/.bashrc>/dev/null)
    then
    read -p "What is the host name or IP of your robot? " answer
    sh -c "echo \"export ROS_MASTER_URI=http://${answer}:11311\" >> ~/.bashrc"
fi
if ! (grep ROS_HOSTNAME ~/.bashrc>/dev/null)
    then
    sh -c "echo \"export ROS_HOSTNAME=`uname -n`.local\" >> ~/.bashrc"
fi
if ! (grep ROSLAUNCH_SSH_UNKNOWN ~/.bashrc>/dev/null)
    then
    sh -c "echo \"export ROSLAUNCH_SSH_UNKNOWN=1\" >> ~/.bashrc"
fi
if ! (grep catkin_ws ~/.bashrc>/dev/null)
    then
    sh -c "echo \"source ~/catkin_ws/devel/setup.bash\" >> ~/.bashrc"
fi

printf "\n${PURPLE}Anytime you want to update ArloBot code from the web you can run this same script again. It will pull down and compile new code without wiping out custom configs in ~/.arlarbot. I run this script myself almost every day.\n"

printf "\n${YELLOW}-----------------------------------${NC}\n"
printf "${YELLOW}ALL DONE! REBOOT AND TRY RVIZ${NC}\n"
printf "${BLUE}Fire up your Arlobot on its machine, and then try these on here:${NC}\n"
printf "${BLUE}~/catkin_ws/src/ArloBot/scripts/view-navigation.sh${NC}\n"
printf "${BLUE}rqt_graph${NC}\n"
