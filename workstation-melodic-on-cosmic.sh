#!/bin/bash
# ROS Melodic "Workstation" Automated Install
# This is to set up enough of ROS to use RVIZ and some other GUI tools,
# on a secondary system. It will not run a robot.

# Run this straight off of github like this:
# bash <(wget -qO- --no-cache https://raw.githubusercontent.com/chrisl8/ArloBot/new-serial-interface/workstation-melodic.sh)

set -e

ROS_RELEASE_NAME=melodic

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

printf "\n${YELLOW}SETTING UP ROS ${ROS_RELEASE_NAME} FOR YOUR REMOTE WORK!${NC}\n"
printf "${YELLOW}-------------------------------------------${NC}\n"
printf "${GREEN}You will be asked for your password for running commands as root!${NC}\n"

version=`lsb_release -sc`

printf "\n${YELLOW}[Checking the Ubuntu version]${NC}\n"
printf "${BLUE}Ubuntu ${version} found${NC}\n"
case ${version} in
  "cosmic")
;;
*)
    printf "${RED}[This script will only work on Ubuntu Cosmic (18.10)]${NC}\n"
exit 1
esac

printf "\n${YELLOW}[Updating & upgrading all existing Ubuntu packages]${NC}\n"
sudo apt update
sudo apt upgrade -y

printf "\n${YELLOW}[Installing additional Ubuntu and ROS Packages for Arlo]${NC}\n"
printf "${BLUE}This runs every time, in case new packages were added.${NC}\n"
# Notes on what the packages are for:
# zbar-tools python-qrtools qtqr for generating and reading QR Codes.
# jq allows shell scripts to read .json formatted config files.
# git allows cloning of source files

sudo apt install -y zbar-tools python-qrtools qtqr jq git

printf "\n${YELLOW}[Cloning or Updating git repositories]${NC}\n"
if ! [[ -d ~/catkin_ws/src ]]; then
    mkdir -p ~/catkin_ws/src
fi
cd ~/catkin_ws/src
if ! [[ -d ~/catkin_ws/src/ArloBot ]]; then
    git clone -b new-serial-interface https://github.com/chrisl8/ArloBot.git
else
    cd ~/catkin_ws/src/ArloBot
    git pull
fi

# This should follow the official ROS install from source instructions closely.
#http://wiki.ros.org/melodic/Installation/Source
# With help from https://stackoverflow.com/a/53382269/4982408
# for issues building Melodic on Ubuntu 18.10
# That is why there is a separate section for extra packages that I need for Arlo above.
# TODO: This does not actually check for new package versions and update,
# TODO: It just runs again in case the last run failed.
# TODO: Actually update if that is possible.
if ! [[ -e ~/ros_catkin_ws/install_isolated/setup.bash ]]; then
    printf "\n${YELLOW}[Installing ROS from Source]${NC}\n"

    sudo apt install -y python-rosdep2 python-rosinstall-generator python-wstool python-rosinstall build-essential
    if ! [[ -e /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
        sudo rosdep init
    fi

    printf "\n${YELLOW}[Running rosdep update]${NC}\n"
    rosdep update
    if ! [[ -d ~/ros_catkin_ws ]]; then
        mkdir ~/ros_catkin_ws
    fi
    cd ~/ros_catkin_ws
    if ! [[ -f src/.rosinstall ]]; then
        ROS_INSTALL_FILE=melodic-desktop-full.rosinstall
        rosinstall_generator desktop_full --rosdistro melodic --deps --tar > ${ROS_INSTALL_FILE}
        # Fix broken ROS downloads
        # See: https://answers.ros.org/question/314827/wstool-failing-with-tf2_msgs/
        # and: https://answers.ros.org/question/315325/wstool-init-fails-on-ros_commrosbag_storage-when-building-melodic/
        sed -i 's/version: \(ros_comm-release-release-melodic-ros_comm\)-[0-9\.\-]\+/version: \1/g' ${ROS_INSTALL_FILE}
        sed -i 's/version: \(ros_comm-release-release-melodic-rosbag_storage\)-[0-9\.\-]\+/version: \1/g' ${ROS_INSTALL_FILE}
        sed -i 's/version: \(ros_comm-release-release-melodic-rosgraph\)-[0-9\.\-]\+/version: \1/g' ${ROS_INSTALL_FILE}
        wstool init -j8 src ${ROS_INSTALL_FILE}

        # Add other packages that are required, but not listed in desktop_full
        rosinstall_generator move_base_msgs --rosdistro melodic --deps --tar > move_base_msgs.rosinstall
        wstool merge -t src move_base_msgs.rosinstall
        wstool update -j8 -t src
    else
        wstool update -j8 -t src
    fi

    # Notice we are fibbing here and forcing "bionic" on "cosmic"
    rosdep install --from-paths src --ignore-src --os=ubuntu:bionic --rosdistro melodic -y

    # Find files where boost calls need to be fixed.
    # See: https://stackoverflow.com/a/53382269/4982408
    #find -type f -print0 | xargs -0 grep 'boost::posix_time::milliseconds' | cut -d: -f1 | sort -u

    # Patch files where boost calls will break compilation due to using floats where only ints are allowed
    # See: https://stackoverflow.com/a/53382269/4982408
    patch ./src/actionlib/include/actionlib/client/simple_action_client.h ~/catkin_ws/src/ArloBot/patches/simple_action_client.patch
    patch ./src/actionlib/include/actionlib/destruction_guard.h ~/catkin_ws/src/ArloBot/patches/destruction_guard.patch
    patch ./src/actionlib/include/actionlib/server/simple_action_server_imp.h ~/catkin_ws/src/ArloBot/patches/simple_action_server_imp.patch
    patch ./src/actionlib/src/connection_monitor.cpp ~/catkin_ws/src/ArloBot/patches/connection_monitor.patch
    patch ./src/actionlib/test/destruction_guard_test.cpp ~/catkin_ws/src/ArloBot/patches/destruction_guard_test.patch

    ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release
fi

# In case .bashrc wasn't set up, or you didn't reboot
if ! (which catkin_make > /dev/null)
    then
    source ~/ros_catkin_ws/install_isolated/setup.bash
fi

if ! [ -d ~/catkin_ws/devel ]
    then
    printf "\n${YELLOW}[Creating the catkin workspace and testing with catkin_make]${NC}\n"
    if ! [[ -d ~/catkin_ws/src ]]; then
        mkdir -p ~/catkin_ws/src
    fi
    cd ~/catkin_ws/src
    catkin_init_workspace
    cd ~/catkin_ws/
    printf "\n${YELLOW}[Building ArloBot Source]${NC}\n"
    catkin_make
    source ${HOME}/catkin_ws/devel/setup.bash
    rospack profile
fi

source ${HOME}/catkin_ws/devel/setup.bash

if ! [[ -f ${HOME}/Desktop/RVIZ.desktop ]]; then
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

if ! [[ -f ${HOME}/.local/share/applications/RVIZ.desktop ]]; then
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

# We will use ~/.arlobot to store "private" data
# That is data that doesn't need to be part of
# the public github repo like user tokens,
# sounds, and room maps and per robot settings
if ! [[ -d ${HOME}/.arlobot ]]; then
    mkdir ${HOME}/.arlobot
fi

ARLOHOME=${HOME}/.arlobot

if ! $(which node); then
    wget -qO- https://raw.githubusercontent.com/creationix/nvm/v0.34.0/install.sh | bash
    export NVM_DIR="${HOME}/.nvm"
    [[ -s "$NVM_DIR/nvm.sh" ]] && . "$NVM_DIR/nvm.sh"  # This loads nvm

    printf "\n${YELLOW}[Initializing the Current Node LTS version]${NC}\n"
    nvm install --lts
    nvm alias default lts/*
fi

if [[ -e ${ARLOHOME}/personalDataForBehavior.json ]]; then
    node ~/catkin_ws/src/ArloBot/node/personalData.js
else
    printf "\n"
    cp ~/catkin_ws/src/ArloBot/scripts/dotarlobot/personalDataForBehavior.json ${ARLOHOME}/
    printf "${GREEN}A brand new ${RED}~/.arlobot/personalDataForBehavior.json${GREEN} file has been created,${NC}\n"
    printf "${LIGHT_PURPLE}Please edit this file to customize according to your robot!${NC}\n"
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
printf "${BLUE}~/catkin_ws/src/ArloBot/scripts/tf2pdf.sh${NC}\n"
printf "${BLUE}rqt_graph${NC}\n"
