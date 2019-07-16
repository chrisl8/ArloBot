#!/bin/bash
# ROS Melodic "Workstation" Automated Install
# This is to set up enough of ROS to use RVIZ and some other GUI tools,
# on a secondary system. It will not run a robot.

# Run this straight off of github like this:
# bash <(wget -qO- --no-cache https://raw.githubusercontent.com/chrisl8/ArloBot/new-serial-interface/workstation-melodic.sh)

#   TESTING
#
# Testing workstation install with Docker:
#
# You can Test this with Docker by installing Docker, then pulling down the Ubuntu 18.10 image:
# sudo docker pull ubuntu:18.10
# cd ~/catkin_ws/src/ArloBot
#
# Then either kick it off all in one shot:
# sudo docker run -ti -v $PWD:/home/user ubuntu:18.10 /bin/bash -c "/home/user/workstation-melodic-on-cosmic.sh"
#
# Or start an interactive shell in Docker and run it, with the ability to make changes and start it again when it finishes:
# sudo docker run -ti -v $PWD:/home/user ubuntu:18.10 /bin/bash
# /home/user/workstation-melodic-on-cosmic.sh
#
# To clean up Docker when you are done run:
# sudo docker system prune
#
# Unfortunately, Travis CI seems to choke on this, so I cannot use CI testing on this script yet.

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

if [[ ! -e /etc/localtime ]]; then
    # These steps are to allow this script to work in a minimal Docker container for testing.
    printf "${YELLOW}[This looks like a Docker setup.]${NC}\n"
    printf "${BLUE}Adding settings and basic packages for Docker based Ubuntu images.${NC}\n"
    # The docker image has no /etc/localtime
    # When the prereq install install the tzdat package,
    # It stops and asks for time zone info.
    # This should prevent that.
    # https://bugs.launchpad.net/ubuntu/+source/tzdata/+bug/1773687
    export DEBIAN_FRONTEND=noninteractive
    # This won't work inside of sudo though, so just install tzdata now
    # rather than letting it get picked up later as a pre-req,
    # and add the other things we know Docker is missing too while we are at it.
    apt update
    apt install -y tzdata sudo lsb-release
    # Now the rest of the script should work as if it was in a normal Ubuntu install.
fi

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
# wget used for grabbing dependencies. Usually already installed, but just in case.

sudo apt install -y zbar-tools python-qrtools qtqr jq git wget

printf "\n${YELLOW}[Cloning or Updating ArloBot Code]${NC}\n"
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
# TODO: http://wiki.ros.org/melodic/Installation/Source#Maintaining_a_Source_Checkout
# TODO: NOTE I may have to check and patch files again?
# TODO: Until I actually SEE updates, it is hard to test the idea.
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
    printf "\n${YELLOW}[Downloading Source Files]${NC}\n"
    if ! [[ -f src/.rosinstall ]]; then
        ROS_INSTALL_FILE=melodic-desktop-full.rosinstall
        rosinstall_generator desktop_full --rosdistro melodic --deps --tar > ${ROS_INSTALL_FILE}

        # TODO: Patching system installed files is probably less than ideal. Other options?
        # NOTE: Another option is to run this script to fix the generated .rosinstall file:
        # https://gist.githubusercontent.com/machinekoder/49ff3aa0734b4b2ec99ec86586cb40c1/raw/1594b0a8f235735c837e4619f21c4bf52d2c9b5e/fix_rosinstall.py
        # from: https://github.com/isaacs/github/issues/1483#issuecomment-464072676
        # but this script requires:
        #sudo apt install python-pip
        #sudo pip install sh
        #wget  https://gist.githubusercontent.com/machinekoder/49ff3aa0734b4b2ec99ec86586cb40c1/raw/1594b0a8f235735c837e4619f21c4bf52d2c9b5e/fix_rosinstall.py
        # and even then it assumes the file is called ros_catkin_ws/docker/kinetic.rosinstall.vanilla

        # Patch wstool for GitHub issue
        # See: https://github.com/vcstools/vcstools/issues/147
        wget https://github.com/vcstools/vcstools/commit/ddcfacb7dd10429ff5e57845d18657c8f1dc2997.patch
        if (sudo patch -N --dry-run --silent /usr/lib/python2.7/dist-packages/vcstools/tar.py ddcfacb7dd10429ff5e57845d18657c8f1dc2997.patch > /dev/null); then
            sudo patch /usr/lib/python2.7/dist-packages/vcstools/tar.py ddcfacb7dd10429ff5e57845d18657c8f1dc2997.patch
        fi
        rm ddcfacb7dd10429ff5e57845d18657c8f1dc2997.patch

        wget https://github.com/vcstools/vcstools/commit/eb39db2a30fdb84506ec14c040f836cce6c8874f.patch
        if (sudo patch -N --dry-run --silent /usr/lib/python2.7/dist-packages/vcstools/tar.py eb39db2a30fdb84506ec14c040f836cce6c8874f.patch > /dev/null); then
            sudo patch /usr/lib/python2.7/dist-packages/vcstools/tar.py eb39db2a30fdb84506ec14c040f836cce6c8874f.patch
        fi
        rm eb39db2a30fdb84506ec14c040f836cce6c8874f.patch

        wstool init -j8 src ${ROS_INSTALL_FILE}

        # Add other packages that are required by ArloBot, but not listed in desktop_full
        # navigation
        ROS_INSTALL_FILE=navigation.rosinstall
        rosinstall_generator navigation --rosdistro melodic --deps --tar > ${ROS_INSTALL_FILE}
        wstool merge --merge-keep -y -t src ${ROS_INSTALL_FILE}
        wstool update -j8 -t src
    else
        # This just runs the update again in case the last one was interrupted or failed due to network issues.
        wstool update -j8 -t src
    fi

    # Notice we are fibbing here and forcing "bionic" on "cosmic"
    rosdep install --from-paths src --ignore-src --os=ubuntu:bionic --rosdistro melodic -y

    # Find files where boost calls need to be fixed.
    # See: https://stackoverflow.com/a/53382269/4982408
    #find -type f -print0 | xargs -0 grep 'boost::posix_time::milliseconds' | cut -d: -f1 | sort -u

    printf "\n${YELLOW}[Patching code to work with Ubuntu 18.10's version of boost]${NC}\n"
    # Patch files where boost calls will break compilation due to using floats where only ints are allowed
    # See: https://stackoverflow.com/a/53382269/4982408
    if (patch -N --dry-run --silent ./src/actionlib/include/actionlib/client/simple_action_client.h ~/catkin_ws/src/ArloBot/patches/simple_action_client.patch > /dev/null); then
        patch ./src/actionlib/include/actionlib/client/simple_action_client.h ~/catkin_ws/src/ArloBot/patches/simple_action_client.patch
    fi

    if (patch -N --dry-run --silent ./src/actionlib/include/actionlib/destruction_guard.h ~/catkin_ws/src/ArloBot/patches/destruction_guard.patch > /dev/null); then
        patch ./src/actionlib/include/actionlib/destruction_guard.h ~/catkin_ws/src/ArloBot/patches/destruction_guard.patch
    fi

    if (patch -N --dry-run --silent ./src/actionlib/include/actionlib/server/simple_action_server_imp.h ~/catkin_ws/src/ArloBot/patches/simple_action_server_imp.patch > /dev/null); then
            patch ./src/actionlib/include/actionlib/server/simple_action_server_imp.h ~/catkin_ws/src/ArloBot/patches/simple_action_server_imp.patch
        fi

    if (patch -N --dry-run --silent ./src/actionlib/src/connection_monitor.cpp ~/catkin_ws/src/ArloBot/patches/connection_monitor.patch > /dev/null); then
        patch ./src/actionlib/src/connection_monitor.cpp ~/catkin_ws/src/ArloBot/patches/connection_monitor.patch
    fi

    if (patch -N --dry-run --silent ./src/actionlib/test/destruction_guard_test.cpp ~/catkin_ws/src/ArloBot/patches/destruction_guard_test.patch > /dev/null); then
        patch ./src/actionlib/test/destruction_guard_test.cpp ~/catkin_ws/src/ArloBot/patches/destruction_guard_test.patch
    fi

    if ! [[ ${TRAVIS} == "true" ]]; then
        printf "\n${YELLOW}[Compiling ROS Source]${NC}\n"
        ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release
    else
        printf "\n${YELLOW}[SKIPPING Compile on Travis CI]${NC}\n"
        printf "${BLUE}Travis CI cannot complete the build. I suspect maybe it chokes on the MD5 processes?${NC}\n"
    fi
fi

if ! [[ ${TRAVIS} == "true" ]]; then
    source ~/ros_catkin_ws/install_isolated/setup.bash
fi

if ! [[ ${TRAVIS} == "true" ]]; then

    if ! [[ -d ~/catkin_ws/devel ]]; then
        printf "\n${YELLOW}[Creating the catkin workspace and testing with catkin_make]${NC}\n"
        if ! [[ -d ~/catkin_ws/src ]]; then
            mkdir -p ~/catkin_ws/src
        fi
        cd ~/catkin_ws/src
        catkin_init_workspace
    fi

    printf "\n${YELLOW}[Building ArloBot Source]${NC}\n"
    cd ~/catkin_ws/
    catkin_make
    source ${HOME}/catkin_ws/devel/setup.bash
    rospack profile
else
    printf "\n${YELLOW}[SKIPPING Compile on Travis CI]${NC}\n"
    printf "${BLUE}Travis CI cannot complete the Arlobot Build because it could not do the ROS build${NC}\n"
fi

if ! [[ -f ${HOME}/Desktop/RVIZ.desktop ]]; then
    printf "\n${YELLOW}[Creating Desktop Icon to run RVIZ]${NC}\n"
    if [[ ! -d ${HOME}/Desktop ]]; then
        mkdir ${HOME}/Desktop
    fi
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

# We will use ~/.arlobot to store "private" data
# That is data that doesn't need to be part of
# the public github repo like user tokens,
# sounds, and room maps and per robot settings
if ! [[ -d ${HOME}/.arlobot ]]; then
    mkdir ${HOME}/.arlobot
fi

ARLO_HOME=${HOME}/.arlobot

if ! (which node > /dev/null); then
    printf "\n${YELLOW}[Installing the Current Node LTS version]${NC}\n"
    # Install nvm
    wget -qO- https://raw.githubusercontent.com/creationix/nvm/v0.34.0/install.sh | bash
    # Initialize nvm without logging out and back in
    export NVM_DIR="${HOME}/.nvm"
    [[ -s "$NVM_DIR/nvm.sh" ]] && . "$NVM_DIR/nvm.sh"  # This loads nvm
    # Install Node LTS version via nvm
    nvm install --lts
    nvm alias default lts/*
fi

if [[ -e ${ARLO_HOME}/personalDataForBehavior.json ]]; then
    node ~/catkin_ws/src/ArloBot/node/personalData.js
else
    printf "\n"
    cp ~/catkin_ws/src/ArloBot/scripts/dotarlobot/personalDataForBehavior.json ${ARLO_HOME}/
    printf "${GREEN}A brand new ${RED}~/.arlobot/personalDataForBehavior.json${GREEN} file has been created,${NC}\n"
    printf "${LIGHT_PURPLE}Please edit this file to customize according to your robot!${NC}\n"
fi

printf "\n${YELLOW}[Setting the ROS environment in your .bashrc file]${NC}\n"
if ! (grep ROS_MASTER_URI ~/.bashrc>/dev/null); then
    if ! [[ ${TRAVIS} == "true" ]]; then
        read -p "What is the host name or IP of your robot? " answer
    else
        # Dummy data for testing
        answer=localhost
    fi
    sh -c "echo \"export ROS_MASTER_URI=http://${answer}:11311\" >> ~/.bashrc"
fi
if ! (grep ROS_HOSTNAME ~/.bashrc>/dev/null); then
    sh -c "echo \"export ROS_HOSTNAME=`uname -n`.local\" >> ~/.bashrc"
fi
if ! (grep ROSLAUNCH_SSH_UNKNOWN ~/.bashrc>/dev/null); then
    sh -c "echo \"export ROSLAUNCH_SSH_UNKNOWN=1\" >> ~/.bashrc"
fi
if ! (grep catkin_ws ~/.bashrc>/dev/null); then
    sh -c "echo \"source ~/catkin_ws/devel/setup.bash\" >> ~/.bashrc"
fi

printf "\n${PURPLE}Anytime you want to update ArloBot code from the web you can run this same script again. It will pull down and compile new code without wiping out custom configs in ~/.arlarbot. I run this script myself almost every day.\n"

printf "\n${YELLOW}-----------------------------------${NC}\n"
printf "${YELLOW}ALL DONE! REBOOT AND TRY RVIZ${NC}\n"
printf "${BLUE}Fire up your Arlobot on its machine, and then try these on here:${NC}\n"
printf "${BLUE}~/catkin_ws/src/ArloBot/scripts/view-navigation.sh${NC}\n"
printf "${BLUE}~/catkin_ws/src/ArloBot/scripts/tf2pdf.sh${NC}\n"
printf "${BLUE}rqt_graph${NC}\n"
