#!/bin/bash

# ROS Indigo Automated Install:

# The BSD License
# Copyright (c) 2014 OROCA and ROS Korea Users Group

echo "Setting up Robot Operating System for your ArloBot!"

echo "You will be asked for your password for running commands via sudo."

version=`lsb_release -sc`

echo "[Checking the ubuntu version]"
case $version in
  "saucy" | "trusty")
  ;;
  *)
    echo "[This script will only work on ubuntu saucy(13.10) or trusty(14.04)]"
    exit 0
esac

echo "[Update & upgrade all existing packages]"
echo "silently . . ."
sudo apt-get update -qq
sudo apt-get upgrade -qq

# I never use this, but if you are having time issues maybe uncomment this.
#echo "[Installing chrony and setting the ntpdate]"
#sudo apt-get install -y chrony
#sudo ntpdate ntp.ubuntu.com

echo "[Checking for ROS repository]"
if [ ! -e /etc/apt/sources.list.d/ros-latest.list ]; then
  echo "[Adding the ROS repository]"
  sudo sh -c "echo \"deb http://packages.ros.org/ros/ubuntu ${version} main\" > /etc/apt/sources.list.d/ros-latest.list"
fi

echo "[Checking the ROS keys]"
roskey=`apt-key list | grep "ROS builder"`
if [ -z "$roskey" ]; then
  echo "[Adding the ROS keys]"
  wget --quiet https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -
fi

echo "[Update & upgrade the packages with the new repository]"
echo "silently . . ."
sudo apt-get update -qq
sudo apt-get upgrade -qq

echo "[Installing ROS!]"
sudo apt-get install -qy ros-indigo-desktop-full ros-indigo-rqt-*
echo "[ROS installed.]"

echo "[rosdep init and python-rosinstall]"
sudo sh -c "rosdep init"
rosdep update
. /opt/ros/indigo/setup.sh
sudo apt-get install -qy python-rosinstall

echo "[Installing ROS Packages for Arlo]"
sudo apt-get install -qy ros-indigo-turtlebot ros-indigo-turtlebot-apps ros-indigo-turtlebot-interactions ros-indigo-turtlebot-simulator ros-indigo-kobuki-ftdi
source /opt/ros/indigo/setup.bash

if ! [ -d ~/catkin_ws/src ]
    then
    echo "[Making the catkin workspace and testing the catkin_make]"
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/src
    catkin_init_workspace
    cd ~/catkin_ws/
    catkin_make
fi

source ~/catkin_ws/devel/setup.bash
rospack profile
cd ~/catkin_ws/src

echo "[Cloning or Updating git repositories]"
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

echo "[Setting the ROS evironment]"
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

# Arlobot Specific settings:

if ! (id|grep dialout>/dev/null)
then
    echo "Adding your user to the dialout group,"
    echo "You may be asked for your password."
    sudo adduser ${USER} dialout
    echo "You may have to reboot before you can use the Propeller Board."
fi

echo "Installing additional required Ubuntu packages for Arlobot"

# For 8-CH USB Relay board:
# Reference: https://code.google.com/p/drcontrol/wiki/Install_RaspberryPi">https://code.google.com/p/drcontrol/wiki/Install_RaspberryPi
# python-ftdi,  python-pip and sudo pip install pylibftdi
# TEST:
#python -m pylibftdi.examples.list_devices
#Should return:
#FTDI:FT245R USB FIFO:A9026EI5
#If you have a USB Relay board attached via USB.

sudo apt-get install -qy python-ftdi python-pip python-serial ros-indigo-openni-* ros-indigo-openni2-* \
    ros-indigo-freenect-* ros-indigo-vision-opencv libopencv-dev python-opencv
# For 8-CH USB Relay board:
sudo pip install pylibftdi

if ! [ -f /etc/udev/rules.d/99-libftdi.rules ]
then
    echo "Adding required sudo rule to reset USB ports."
    echo "You may be asked for your password."
    sudo ~/catkin_ws/src/ArloBot/addRuleForUSBRelayBoard.sh
    echo "You may have to reboot before the USB Relay board will function!"
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

if [ ! -e ${ARLOHOME}/arlobot.yaml]
    then
    cp ${HOME}/catkin_ws/src/ArloBot/src/arlobot/arlobot_bringup/param/arlobot.yaml ${ARLOHOME}/
else
    diff -u ${HOME}/catkin_ws/src/ArloBot/src/arlobot/arlobot_bringup/param/arlobot.yaml ${ARLOHOME}/
    cp -i ${HOME}/catkin_ws/src/ArloBot/src/arlobot/arlobot_bringup/param/arlobot.yaml ${ARLOHOME}/
fi

echo "All done! Reboot and start testing!"
echo "Look at README.md for testing ideas."
