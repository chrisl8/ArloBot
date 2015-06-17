#!/bin/bash
# ROS Indigo Automated Install - credit goes to everyone
# Blame goes to ChrisL8

# Run this straight off of github like this:
# bash <(wget -qO- https://raw.githubusercontent.com/chrisl8/ArloBot/master/setup.sh)

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

printf "\n${YELLOW}Setting up Robot Operating System for your ArloBot!\n\n"
printf "${GREEN}You will be asked for your password for running commands as root!\n"

version=`lsb_release -sc`

printf "[Checking the ubuntu version]\n"
case $version in
  "saucy" | "trusty")
;;
*)
printf "[This script will only work on ubuntu saucy(13.10) or trusty(14.04)]\n"
exit 0
esac

printf "[Updating & upgrading all existing Ubuntu packages]\n"
printf "silently . . .\n"
# NOTE: You have to pipe /dev/null INTO apt-get to make it work from wget.
sudo apt-get update -qq < /dev/null
sudo apt-get upgrade -qq < /dev/null

# I never use this, but if you are having time issues maybe uncomment this.
#printf "[Installing chrony and setting the ntpdate]\n"
#sudo apt-get install -y chrony
#sudo ntpdate ntp.ubuntu.com

printf "[Checking for ROS repository]\n"
if ! [ -e /etc/apt/sources.list.d/ros-latest.list ]
    then
    printf "[Adding the ROS repository]\n"
    sudo sh -c "echo \"deb http://packages.ros.org/ros/ubuntu ${version} main\" > /etc/apt/sources.list.d/ros-latest.list"
    printf "[Checking the ROS keys]\n"
    roskey=`apt-key list | grep -i "ROS builder"`
    if [ -z "$roskey" ]
        then
        printf "[Adding the ROS keys]\n"
        wget --quiet https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -
        printf "[Update & upgrade the packages again with the new repository]\n"
        printf "silently . . .\n"
        sudo apt-get update -qq < /dev/null
        sudo apt-get upgrade -qq < /dev/null
    fi
fi

printf "[Installing ROS!]\n"
sudo apt-get install -qy ros-indigo-desktop-full ros-indigo-rqt-* < /dev/null
printf "[ROS installed.]\n"

printf "[rosdep init and python-rosinstall]\n"
if ! [ -e /etc/ros/rosdep/sources.list.d/20-default.list ]
    then
    sudo sh -c "rosdep init"
fi
rosdep update
source /opt/ros/indigo/setup.bash
sudo apt-get install -qy python-rosinstall < /dev/null

printf "[Installing ROS Packages for Arlo]\n"
sudo apt-get install -qy ros-indigo-turtlebot ros-indigo-turtlebot-apps ros-indigo-turtlebot-interactions ros-indigo-turtlebot-simulator ros-indigo-kobuki-ftdi < /dev/null
source /opt/ros/indigo/setup.bash

if ! [ -d ~/catkin_ws/src ]
    then
    printf "[Making the catkin workspace and testing the catkin_make]\n"
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/src
    catkin_init_workspace
    cd ~/catkin_ws/
    catkin_make
fi

source ~/catkin_ws/devel/setup.bash
rospack profile
cd ~/catkin_ws/src

printf "[Cloning or Updating git repositories]\n"
if ! [ -d ~/catkin_ws/src/hector_slam ]
    then
    git clone https://github.com/chrisl8/hector_slam.git
else
    cd ~/catkin_ws/src/hector_slam
    git pull
fi

if ! [ -d ~/catkin_ws/src/hector_navigation ]
    then
    git clone https://github.com/chrisl8/hector_navigation.git
else
    cd ~/catkin_ws/src/hector_navigation
    git pull
fi

if ! [ -d ~/catkin_ws/src/ArloBot ]
    then
    git clone https://github.com/chrisl8/ArloBot.git
else
    cd ~/catkin_ws/src/ArloBot
    git pull
fi

# Optionally:
if ! [ -d ~/catkin_ws/src/Metatron ]
    then
    git clone https://github.com/chrisl8/Metatron.git
else
    cd ~/catkin_ws/src/Metatron
    git pull
fi

# If you have an XV-11 "Neato" Scanner
if ! [ -d ~/catkin_ws/src/xv_11_laser_driver ]
    then
    git clone https://github.com/chrisl8/xv_11_laser_driver.git
else
    cd ~/catkin_ws/src/xv_11_laser_driver
    git pull
fi

# If you have the excellent ROS by Example book now is a good time to clone the code for following along in the book:
if ! [ -d ~/catkin_ws/src/rbx1 ]
    then
    git clone -b indigo-devel https://github.com/pirobot/rbx1.git
else
    cd ~/catkin_ws/src/rbx1
    git pull
fi

# If you want to use the USB Camera code from the ROS by Example book:
if ! [ -d ~/catkin_ws/src/usb_cam ]
    then
    git clone https://github.com/bosch-ros-pkg/usb_cam.git
else
    cd ~/catkin_ws/src/usb_cam
    git pull
fi

cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash
rospack profile

printf "[Setting the ROS evironment]\n"
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

# Run Metatron Setup Script:
~/catkin_ws/src/Metatron/scripts/setup.sh

# Arlobot Specific settings:

if ! (id|grep dialout>/dev/null)
    then
    printf "Adding your user to the dialout group,\n"
    printf "You may be asked for your password.\n"
    sudo adduser ${USER} dialout
    printf "You may have to reboot before you can use the Propeller Board.\n"
fi

printf "Installing additional required Ubuntu packages for Arlobot\n"

# For 8-CH USB Relay board:
# Reference: https://code.google.com/p/drcontrol/wiki/Install_RaspberryPi">https://code.google.com/p/drcontrol/wiki/Install_RaspberryPi
# python-ftdi,  python-pip and sudo pip install pylibftdi
# TEST:
#python -m pylibftdi.examples.list_devices
#Should return:
#FTDI:FT245R USB FIFO:A9026EI5
#If you have a USB Relay board attached via USB.

sudo apt-get install -qy python-ftdi python-pip python-serial ros-indigo-openni-* ros-indigo-openni2-* ros-indigo-freenect-* ros-indigo-vision-opencv libopencv-dev python-opencv < /dev/null
# For 8-CH USB Relay board:
sudo pip install pylibftdi

if ! [ -f /etc/udev/rules.d/99-libftdi.rules ]
    then
    printf "Adding required sudo rule to reset USB ports."
    printf "You may be asked for your password."
    sudo ~/catkin_ws/src/ArloBot/addRuleForUSBRelayBoard.sh
    printf "You may have to reboot before the USB Relay board will function!"
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
    if ! (diff ${HOME}/catkin_ws/src/ArloBot/src/arlobot/arlobot_bringup/param/arlobot.yaml ${ARLOHOME}/arlobot.yaml)
        then
        printf "\nThe arlobot.yaml file in the repository is different from the one\n"
        printf "in your local settings.\n"
        printf "This is expected, but just in case, please look over the differences,\n"
        printf "and see if you need to copy in any new settings, or overwrite the file completely:\n"
        diff ${HOME}/catkin_ws/src/ArloBot/src/arlobot/arlobot_bringup/param/arlobot.yaml ${ARLOHOME}/arlobot.yaml
        cp -i ${HOME}/catkin_ws/src/ArloBot/src/arlobot/arlobot_bringup/param/arlobot.yaml ${ARLOHOME}/
        printf "\n"
    fi
else
    printf "\n"
    cp ${HOME}/catkin_ws/src/ArloBot/src/arlobot/arlobot_bringup/param/arlobot.yaml ${ARLOHOME}/
    printf "A brand new ~/.arlobot/arlobot.yaml file has been created,\n"
    printf "please edit this file to customize according to your robot!\n\n"
fi

printf "All done! Reboot and start testing!\n"
printf "Look at README.md for testing ideas.\n\n"
printf "See here for your next step:\n"
printf "http://ekpyroticfrood.net/?p=165\n\n"
