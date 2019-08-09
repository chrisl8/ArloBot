#!/usr/bin/env bash
# shellcheck disable=SC2059 disable=SC2129
# ROS Arlobot Automated Install

INSTALLING_ROS_DISTRO=kinetic

# Run this straight off of github like this:
# bash <(wget -qO- --no-cache https://raw.githubusercontent.com/chrisl8/ArloBot/new-serial-interface/setup-kinetic.sh)

#   TESTING
#
# Testing workstation install with Docker:
#
# You can Test this with Docker by installing Docker, then pulling down the Ubuntu 16.04 image:
# sudo docker pull ubuntu:16.04
# cd ~/catkin_ws/src/ArloBot
#
# Then either kick it off all in one shot:
# sudo docker run -ti -v $PWD:/home/user ubuntu:16.04 /bin/bash -c "/home/user/setup-kinetic.sh"
#
# Or start an interactive shell in Docker and run it, with the ability to make changes and start it again when it finishes:
# sudo docker run -ti -v $PWD:/home/user ubuntu:16.04 /bin/bash
# /home/user/setup-kinetic.sh
#
# If you started a non-interactive ("one shot") build and then it crashed and you want to get in and look around:
# https://docs.docker.com/engine/reference/commandline/commit/
# Find the name of the container:
# sudo docker ps -a
# sudo docker commit $CONTAINER_ID mytestimage
# sudo docker run -ti -v $PWD:/home/user mytestimage /bin/bash
#
# and when you are done delete the image:
# sudo docker image rm mytestimage
#
# Then you can look around and try running the script if you want again.
#
#
# To clean up Docker when you are done run:
# sudo docker system prune
#
# Also note that if you add --rm to the run command on any docker command above, it will automatically remove the container
# after you leave it, instead of leaving it hanging around.

set -e

BLUE='\033[0;34m'
GREEN='\033[0;32m'
RED='\033[0;31m'
PURPLE='\033[0;35m'
LIGHT_PURPLE='\033[1;35m'
YELLOW='\033[1;33m'
LIGHTCYAN='\033[1;36m'
LIGHTBLUE='\033[1;34m'
LIGHTPURPLE='\033[1;35m'
NC='\033[0m' # NoColor

function finish() {
  if [[ -z ${INSTALL_FINISHED} ]]; then
    printf "\n"
    printf "${RED}INSTALL FAILURE!!!${NC}\n"
    printf "${RED}The Install Script has failed. Please investigate cause, correct, and run again before proceeding.${NC}\n"
    printf "\n"
    printf "${YELLOW}If this was a transient error, such as a network failure connecting to something, you may just need to run it again.${NC}\n"
    printf "\n"
    exit 1
  fi
}
trap finish EXIT

printf "\n${YELLOW}SETTING UP ROS ${INSTALLING_ROS_DISTRO} FOR YOUR ARLOBOT!${NC}\n"
printf "${YELLOW}---------------------------------------------------${NC}\n"
printf "${GREEN}You will be asked for your password for running commands as root!${NC}\n"

DOCKER_TEST_INSTALL=false
if [[ ! -e /etc/localtime ]]; then
  # These steps are to allow this script to work in a minimal Docker container for testing.
  printf "${YELLOW}[This looks like a Docker setup.]${NC}\n"
  printf "${BLUE}Adding settings and basic packages for Docker based Ubuntu images.${NC}\n"
  # The docker image has no /etc/localtime
  # When the prereq install installs the tzdat package,
  # It stops and asks for time zone info.
  # This should prevent that.
  # https://bugs.launchpad.net/ubuntu/+source/tzdata/+bug/1773687
  export DEBIAN_FRONTEND=noninteractive
  # This won't work inside of sudo though, so just install tzdata now
  # rather than letting it get picked up later as a pre-req,
  # and add the other things we know Docker is missing too while we are at it.
  apt update
  apt install -y tzdata sudo lsb-release gnupg cron
  # Now the rest of the script should work as if it was in a normal Ubuntu install.
  DOCKER_TEST_INSTALL=true
fi

version=$(lsb_release -sc)

printf "\n${YELLOW}[Checking the Ubuntu version]${NC}\n"
printf "${BLUE}Ubuntu ${version} found${NC}\n"
case ${version} in
"xenial") ;;

*)
  printf "${RED}[This script will only work on Ubuntu Xenial (16.04)]${NC}\n"
  exit 1
  ;;
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
  export APT_KEY_DONT_WARN_ON_DANGEROUS_USAGE=DontWarn
  if ! apt-key list | grep -i "ROS builder"; then
    printf "${BLUE}[Adding the ROS keys]${NC}\n"
    # The pool options are listed here: https://sks-keyservers.net/overview-of-pools.php
    APT_KEY_SERVER=pool.sks-keyservers.net
    COMMAND_DONE=1
    COMMAND_LOOPS=0
    while [[ ${COMMAND_DONE} -gt 0 ]]; do
      if [[ ${COMMAND_LOOPS} -gt 10 ]]; then
        printf "${RED}Too many retires attempting to get ROS apt key.${NC}\n"
        rm /etc/apt/sources.list.d/ros-latest.list
        exit 1
      fi
      if [[ ${COMMAND_LOOPS} -gt 0 ]]; then
        printf "${RED}Failed to retrieve ROS apt key. Retrying...${NC}\n"
        sleep 5
      fi
      sudo apt-key adv --keyserver hkp://${APT_KEY_SERVER}:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && COMMAND_DONE=$?
      COMMAND_LOOPS=$((COMMAND_LOOPS + 1))
    done
    printf "${BLUE}Finished adding keys for ROS install sources.${NC}\n"
  fi
fi

printf "\n${YELLOW}[Updating & upgrading all existing Ubuntu packages]${NC}\n"
sudo apt update
sudo apt upgrade -y

# This should follow the official ROS install instructions closely.
# That is why there is a separate section for extra packages that I need for Arlo.
if ! (dpkg -s ros-${INSTALLING_ROS_DISTRO}-desktop-full | grep "Status: install ok installed" &>/dev/null); then
  printf "\n${YELLOW}[Installing ROS]${NC}\n"
  sudo apt install -y ros-${INSTALLING_ROS_DISTRO}-desktop-full
  printf "${YELLOW}[ROS installed!]${NC}\n"
  printf "\n${YELLOW}[rosdep init and python-rosinstall]${NC}\n"
  if ! [[ -e /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    sudo sh -c "rosdep init"
  fi
  printf "${BLUE}Running rosdep update . . .${NC}\n"
  rosdep update
  # shellcheck source=/opt/ros/kinetic/setup.bash
  source /opt/ros/${INSTALLING_ROS_DISTRO}/setup.bash
  printf "\n${BLUE}Installing ROS Dependencies for building packages${NC}\n"
  sudo apt install -y python-rosinstall python-rosinstall-generator python-wstool build-essential
  # END Official ROS Install section
fi

# In case .bashrc wasn't set up, or you didn't reboot
if ! (command -v catkin_make >/dev/null); then
  # shellcheck source=/opt/ros/kinetic/setup.bash
  source /opt/ros/${INSTALLING_ROS_DISTRO}/setup.bash
fi

printf "\n${YELLOW}[Installing additional Ubuntu and ROS Packages for Arlo]${NC}\n"
printf "${BLUE}This runs every time, in case new packages were added.${NC}\n"
# ### Notes on what the packages are for ###
# python-serial - required for ROS to talk to the Propeller board
# python-ftdi, python-pip, libftdi-dev - required by pylibftdi for talking to USB based serial boards like relay boards, etc.
# https://pylibftdi.readthedocs.io/en/0.15.0/installation.html
# For 8-CH USB Relay board:
# Reference: https://code.google.com/p/drcontrol/wiki/Install_RaspberryPi">https://code.google.com/p/drcontrol/wiki/Install_RaspberryPi
# TEST:
#python -m pylibftdi.examples.list_devices
#Should return:
#FTDI:FT245R USB FIFO:A9026EI5
#If you have a USB Relay board attached via USB.
# expect-dev required to get 'unbuffer' which is required by node to spawn ROS commands and get real time stdout data
# http://stackoverflow.com/a/11337310
# http://linux.die.net/man/1/unbuffer
# jq - allows shell scripts to read .json formatted config files.
# festival and fsetvox-en1 are for text to speech
# libav-tools is for ffmpeg to stream audio to another system.
# zbar-tools - reading QR codes.
# libftdi1 - required by SimpleIDE for the Parallax Propeller board
# libqtgui4 - Required by simpleide
# libqtcore4 - Required by simpleide
# libgif-dev - required for roslib in order to build canvas
# rtabmap - for 3D mapping RTABMAP
# pulseaudio & pavucontrol - for setting the default microphone. I use this for mycroft among other things
# ros-${INSTALLING_ROS_DISTRO}-pointcloud-to-laserscan - for Scanse Sweep
# git - allows for cloning of repositories
# ros-${INSTALLING_ROS_DISTRO}-geodesy for hector compile, might not need it if I stop using hector_explore
# libceres-dev for hector compile, might not need it if I stop using hector_explore
# xvfb libgtk2.0-0 libnotify-dev libgconf-2-4 libnss3 libxss1 libasound2 - For Cypress Testing https://docs.cypress.io/guides/guides/continuous-integration.html#Advanced-setup

PACKAGE_TO_INSTALL_LIST=("ros-${INSTALLING_ROS_DISTRO}-rqt-*" "ros-${INSTALLING_ROS_DISTRO}-kobuki-ftdi" python-ftdi1 python-pip python-serial "ros-${INSTALLING_ROS_DISTRO}-openni-*" "ros-${INSTALLING_ROS_DISTRO}-openni2-*" "ros-${INSTALLING_ROS_DISTRO}-vision-opencv" "ros-${INSTALLING_ROS_DISTRO}-rtabmap-ros" "ros-${INSTALLING_ROS_DISTRO}-scan-tools" "ros-${INSTALLING_ROS_DISTRO}-freenect-*" "ros-${INSTALLING_ROS_DISTRO}-explore-lite" libopencv-dev python-opencv "ros-${INSTALLING_ROS_DISTRO}-rosbridge-server" "ros-${INSTALLING_ROS_DISTRO}-tf2-tools" imagemagick fswebcam festival festvox-en1 libv4l-dev jq expect-dev curl libav-tools zbar-tools openssh-server libftdi-dev libftdi1 libgif-dev pulseaudio pavucontrol "ros-${INSTALLING_ROS_DISTRO}-pointcloud-to-laserscan" git libqtgui4 libqtcore4 xvfb libgtk2.0-0 libnotify-dev libgconf-2-4 libnss3 libxss1 libasound2 espeak)

if ! [[ ${TRAVIS} == "true" ]] && ! [[ ${DOCKER_TEST_INSTALL} == "true" ]]; then
  PACKAGE_TO_INSTALL_LIST=("${PACKAGE_TO_INSTALL_LIST[@]}" "ros-${INSTALLING_ROS_DISTRO}-turtlebot-apps" "ros-${INSTALLING_ROS_DISTRO}-turtlebot-interactions" "ros-${INSTALLING_ROS_DISTRO}-turtlebot-simulator")
else
  printf "\n${GREEN}Skipping turtlebot bits forTravis CI Testing, because librealsense fails due to uvcvideo in Travis CI environment${NC}\n"
fi
sudo apt install -y "${PACKAGE_TO_INSTALL_LIST[@]}"

# Update pip?
sudo -H pip install --upgrade pip
if [[ -z ${USER} ]]; then
  # This probably only happens in Docker, but note that it isn't just for my own purpose
  # that I need $USER, npm uses it too, else you get weird errors like:
  #sh: 1: cannot create build_log.txt: Permission denied
  # even when running as root! if $USER is not set
  USER=$(whoami)
  export USER
fi
if [[ -d ${HOME}/.cache/ ]]; then
  sudo chown -R "${USER}" "${HOME}/.cache/"
fi

# As of 4/27/2016 Rosbridge required me to install twisted via pip otherwise it failed.
sudo -H pip install twisted

if ! [[ -d ~/catkin_ws/src ]]; then
  printf "\n${YELLOW}[Creating the catkin workspace and testing with catkin_make]${NC}\n"
  mkdir -p ~/catkin_ws/src
  cd ~/catkin_ws/src
  catkin_init_workspace
  cd ~/catkin_ws/
  catkin_make
  # shellcheck source=/home/chrisl8/catkin_ws/devel/setup.bash
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

printf "${BLUE}ArloBot respository${NC}\n"
if ! [[ -d ~/catkin_ws/src/ArloBot ]]; then
  git clone -b new-serial-interface https://github.com/chrisl8/ArloBot.git
else
  cd ~/catkin_ws/src/ArloBot
  git pull
fi

printf "\n${BLUE}Neato XV11 respository${NC}\n"
# Only needed if you have an XV-11 "Neato" Scanner
cd ~/catkin_ws/src
if ! [[ -d ~/catkin_ws/src/xv_11_laser_driver ]]; then
  git clone https://github.com/chrisl8/xv_11_laser_driver.git
else
  cd ~/catkin_ws/src/xv_11_laser_driver
  git pull
fi

printf "\n${BLUE}Scanse Sweep respository${NC}\n"
# Only needed if you have a Scanse Sweep, but it doesn't hurt.
if ! [[ -f /usr/local/lib/cmake/sweep/SweepConfig.cmake ]]; then
  cd
  git clone https://github.com/scanse/sweep-sdk.git
  cd "${HOME}/sweep-sdk/libsweep"
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

printf "\n${BLUE}ROS by Example code${NC}\n"
cd ~/catkin_ws/src
# If you have the excellent ROS by Example book now is a good time to clone the code for following along in the book:
if ! [[ -d ~/catkin_ws/src/rbx1 ]]; then
  git clone -b indigo-devel https://github.com/pirobot/rbx1.git
else
  cd ~/catkin_ws/src/rbx1
  git pull
fi

printf "\n${BLUE}USB Web Cam code for ROS by Example${NC}\n"
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
    printf "\n${YELLOW}Do you want to install Mycroft on the Robot?${NC}\n"
    printf "${BLUE}Mycroft can be used to talk to your robot, and have it talk to you.${NC}\n"
    printf "${BLUE}Mycroft can be heavy on system resources on older systems though.${NC}\n\n"
    read -n 1 -s -r -p "Press 'y' to install Mycroft" RESPONSE_TO_MYCROFT_QUERY
    echo ""

    if [[ "${RESPONSE_TO_MYCROFT_QUERY}" == "y" ]]; then
      git clone -b master https://github.com/MycroftAI/mycroft-core.git
      cd ~/catkin_ws/src/ArloBot/mycroft-core
      printf "\n${BLUE}[There will be a lot of questions. I answer Yes to all of them personally.]${NC}\n"
      ./dev_setup.sh
      ./start-mycroft.sh all
      printf "\n${YELLOW}Giving Mycoroft time to download skills.${NC}\n"
      #sleep 60
      #./stop-mycroft.sh
      #cd mycroft/tts/
      #ln -s ${HOME}/catkin_ws/src/ArloBot/mycroft-things/arlobot_tts.py

      printf "\n${YELLOW}[IF you want to use Mycroft:]${NC}\n"
      printf "\n${YELLOW}[Then see https://docs.mycroft.ai/development/cerberus for configuration info.]${NC}\n"
      printf "\n${YELLOW}[See more info at: https://docs.mycroft.ai/installing.and.running/installation/git.clone.install]${NC}\n"
      printf "\n${YELLOW}[At the least you will have to register Mycroft if you want full functionality, althoug it does work without registering.]${NC}\n"
    fi
  else
    printf "\n${YELLOW}[Updating Mycroft]${NC}\n"
    cd ~/catkin_ws/src/ArloBot/mycroft-core
    ./stop-mycroft.sh || true # Do not let failures crash the script.
    git pull
    ./dev_setup.sh
    ./start-mycroft.sh all
  fi
  #cd ~/catkin_ws/src/ArloBot/mycroft-core
  #printf "\n${YELLO}Patching Mycroft TTS to include Arlobot TTS if we want it.{NC}\n"
  # git diff __init__.py > ~/catkin_ws/src/ArloBot/mycroft-things/tts_source_patch.diff
  #git apply ~/catkin_ws/src/ArloBot/mycroft-things/tts_source_patch.diff
else
  printf "\n${GREEN}Skipping Mycroft entirely for Travis CI Testing${NC}\n"
  # ./dev_setup.sh asks interactive questions!
fi

if [[ -d /opt/mycroft/skills ]]; then
  printf "${BLUE}Updating ArloBot Mycroft Skills${NC}\n"
  if ! [[ -L /opt/mycroft/skills/arlobot-robot-skill ]]; then
    cd /opt/mycroft/skills/
    ln -s "${HOME}/catkin_ws/src/ArloBot/mycroft-arlobot-skill" arlobot-robot-skill
  fi
  if ! [[ -L /opt/mycroft/skills/arlobot-smalltalk-skill ]]; then
    cd /opt/mycroft/skills/
    ln -s "${HOME}/catkin_ws/src/ArloBot/mycroft-smalltalk-skill" arlobot-smalltalk-skill
  fi
fi

printf "\n${YELLOW}[(Re)Building ROS Source files.]${NC}\n"
cd ~/catkin_ws
catkin_make
# shellcheck source=/home/chrisl8/catkin_ws/devel/setup.bash
source ~/catkin_ws/devel/setup.bash
rospack profile

if ! (grep ROS_HOSTNAME ~/.bashrc >/dev/null); then
  printf "\n${YELLOW}[Setting the ROS_HOSTNAME in your .bashrc file]${NC}\n"
  sh -c "echo \"export ROS_HOSTNAME=$(uname -n)\" >> ~/.bashrc"
fi
if ! (grep ROSLAUNCH_SSH_UNKNOWN ~/.bashrc >/dev/null); then
  printf "\n${YELLOW}[Setting the ROSLAUNCH_SSH_UNKNOWN in your .bashrc file]${NC}\n"
  sh -c "echo \"export ROSLAUNCH_SSH_UNKNOWN=1\" >> ~/.bashrc"
fi
if ! (grep catkin_ws ~/.bashrc >/dev/null); then
  printf "\n${YELLOW}[Setting the ROS setup.bash source call in your .bashrc file]${NC}\n"
  sh -c "echo \"source ~/catkin_ws/devel/setup.bash\" >> ~/.bashrc"
fi

# For 8-CH USB Relay board:
if [[ ${TRAVIS} == "true" ]]; then
  pip install --user pylibftdi
else
  pip install pylibftdi
fi
# Required by pylibftdi
# https://pylibftdi.readthedocs.io/en/0.15.0/installation.html
if ! [[ -f /etc/udev/rules.d/99-libftdi.rules ]]; then
  printf "\n${YELLOW}[Adding required sudo rule for pylibftdi to access USB based serial ports.]${NC}\n"
  sudo "${HOME}/catkin_ws/src/ArloBot/scripts/addRuleForUSBRelayBoard.sh"
  printf "${RED}You may have to reboot before the USB Relay board will function!${NC}\n"
fi

printf "\n${YELLOW}[Installing and Initializing the Current Node LTS version]${NC}\n"

printf "${BLUE}[Installing/Updating Node Version Manager]${NC}\n"
if [[ -e ${HOME}/.nvm/nvm.sh ]]; then
  printf "${BLUE}Deactivating existing Node Version Manager:${NC}\n"
  export NVM_DIR="${HOME}/.nvm"
  # shellcheck source=/home/chrisl8/.nvm/nvm.sh
  [[ -s "$NVM_DIR/nvm.sh" ]] && . "$NVM_DIR/nvm.sh" # This loads nvm
  nvm deactivate
fi

wget -qO- https://raw.githubusercontent.com/creationix/nvm/v0.34.0/install.sh | bash
export NVM_DIR="${HOME}/.nvm"
# shellcheck source=/home/chrisl8/.nvm/nvm.sh
[[ -s "$NVM_DIR/nvm.sh" ]] && . "$NVM_DIR/nvm.sh" # This loads nvm

export NVM_SYMLINK_CURRENT=true
if ! (grep NVM_SYMLINK_CURRENT ~/.bashrc >/dev/null); then
  printf "\n${YELLOW}[Setting the NVM current environment in your .bashrc file]${NC}\n"
  sh -c "echo \"export NVM_SYMLINK_CURRENT=true\" >> ~/.bashrc"
fi
nvm install --lts
nvm alias default lts/*

printf "\n${YELLOW}[Updating npm]${NC}\n"
npm update -g npm

printf "\n${YELLOW}[Grabbing/Updating global dependencies for node packages]${NC}\n"
printf "${BLUE}You may get some errors here, that is normal. As long as things work, it is OK.$NC\n"
cd
npm install -g pm2
if ! (command -v log.io-harvester >/dev/null) && ! [[ ${DOCKER_TEST_INSTALL} == "true" ]]; then
  # Does not work in Docker (testing) on Ubuntu 18.04
  # No need to update it since it is basically static now.
  npm install -g https://github.com/pruge/Log.io
fi
cd "${HOME}/catkin_ws/src/ArloBot/node"
printf "\n${YELLOW}[Grabbing node dependencies for scripts]${NC}\n"
printf "${BLUE}You may get some errors here, that is normal. As long as things work, it is OK.$NC\n"
npm ci

cd "${HOME}/catkin_ws/src/ArloBot/website"
printf "\n${YELLOW}[Grabbing node dependencies for React website]${NC}\n"
npm ci
printf "\n${YELLOW}[Building React website]${NC}\n"
npm run build

cd "${HOME}/catkin_ws/src/ArloBot/cypress-tests"
printf "\n${YELLOW}[Installing Cypress.io for Tests]$NC\n"
npm ci

if ! (command -v mjpg_streamer >/dev/null); then
  printf "\n${YELLOW}[Installing mjpg_streamer for Web Page camera viewing]${NC}\n"
  cd "${HOME}/catkin_ws/src/ArloBot/"
  git clone https://github.com/jacksonliam/mjpg-streamer.git
  cd ${HOME}/catkin_ws/src/ArloBot/mjpg-streamer/mjpg-streamer-experimental
  make distclean
  make
  sudo make install
  # See scripts/streamVideoTest.sh for details on mjpg_streamer usage.
fi

printf "\n${YELLOW}[Enable non-root use of Bluetooth 4.0.]${NC}\n"
sudo setcap cap_net_raw+eip "$(eval readlink -f "$(command -v node)")"

if ! [[ -f ${HOME}/Desktop/arlobot.desktop ]]; then
  printf "\n${YELLOW}[Creating Desktop Icon]${NC}\n"
  if [[ ! -d ${HOME}/Desktop ]]; then
    mkdir "${HOME}/Desktop"
  fi
  echo "[Desktop Entry]" >"${HOME}/Desktop/arlobot.desktop"
  echo "Encoding=UTF-8" >>"${HOME}/Desktop/arlobot.desktop"
  echo "Name=ArloBot" >>"${HOME}/Desktop/arlobot.desktop"
  echo "GenericName=ArloBot" >>"${HOME}/Desktop/arlobot.desktop"
  echo "Comment=Start the robot" >>"${HOME}/Desktop/arlobot.desktop"
  if (command -v lxterminal >/dev/null); then
    echo "Exec=lxterminal --command \"${HOME}/catkin_ws/src/ArloBot/scripts/arlobotXwindows.sh\"" >>"${HOME}/Desktop/arlobot.desktop"
  elif (command -v gnome-terminal >/dev/null); then
    echo "Exec=gnome-terminal --command \"${HOME}/catkin_ws/src/ArloBot/scripts/arlobotXwindows.sh\"" >>"${HOME}/Desktop/arlobot.desktop"
  fi
  echo "Icon=${HOME}/catkin_ws/src/ArloBot/icon-70x70.png" >>"${HOME}/Desktop/arlobot.desktop"
  echo "Type=Application" >>"${HOME}/Desktop/arlobot.desktop"
  echo "Path=${HOME}/catkin_ws/src/ArloBot/scripts/" >>"${HOME}/Desktop/arlobot.desktop"
  echo "Terminal=false" >>"${HOME}/Desktop/arlobot.desktop"
  chmod +x "${HOME}/Desktop/arlobot.desktop"
fi

if [[ ! -d ${HOME}/.arlobot ]]; then
  printf "\n${YELLOW}[Setting up .arlobot folder]${NC}\n"
  printf "${GREEN}This holds personal data for your robot.${NC}\n"
  # We will use ~/.arlobot to store "private" data
  # That is data that doesn't need to be part of
  # the public github repo like user tokens,
  # sounds, and room maps and per robot settings
  mkdir "${HOME}/.arlobot"
fi

ARLO_HOME=${HOME}/.arlobot

if [[ -e ${ARLO_HOME}/personalDataForBehavior.json ]]; then
  node "${HOME}/catkin_ws/src/ArloBot/node/personalData.js"
else
  printf "\n"
  cp "${HOME}/catkin_ws/src/ArloBot/scripts/dotarlobot/personalDataForBehavior.json" "${ARLO_HOME}/"
  printf "${GREEN}A brand new ${RED}${ARLO_HOME}/personalDataForBehavior.json${GREEN} file has been created,${NC}\n"
  printf "${LIGHT_PURPLE}Please edit this file to customize according to your robot!${NC}\n"
fi

if [[ ! -d ${ARLO_HOME}/rosmaps ]]; then
  mkdir "${ARLO_HOME}/rosmaps"
fi

if [[ ! -d ${ARLO_HOME}/sounds ]]; then
  mkdir "${ARLO_HOME}/sounds"
fi

if [[ ! -d ${ARLO_HOME}/status ]]; then
  mkdir "${ARLO_HOME}/status"
fi
sudo chmod -R 777 "${ARLO_HOME}/status"

if [[ ! -d ${ARLO_HOME}/status/doors ]]; then
  mkdir "${ARLO_HOME}/status/doors"
fi
sudo chmod -R 777 "${ARLO_HOME}/status/doors"

if ! (id | grep dialout >/dev/null); then
  printf "\n${GREEN}Adding your user to the 'dialout' group for serial port access.${NC}\n"
  sudo adduser "${USER}" dialout >/dev/null
  printf "${RED}You may have to reboot before you can use the Propeller Board.${NC}\n"
fi

if ! (id | grep video >/dev/null); then
  printf "\n${GREEN}Adding your user to the 'video' group for access to cameras.${NC}\n"
  sudo adduser "${USER}" video >/dev/null
fi

if ! (command -v simpleide >/dev/null); then
  printf "\n${YELLOW}[Setting up Parallax SimpleIDE for putting code on Activity Board.]${NC}\n"
  cd /tmp
  wget https://web.archive.org/web/20161005174013/http://downloads.parallax.com/plx/software/side/101rc1/simple-ide_1-0-1-rc1_amd64.deb
  sudo dpkg -i /tmp/simple-ide_1-0-1-rc1_amd64.deb
  rm /tmp/simple-ide_1-0-1-rc1_amd64.deb
fi

if ! [[ -e ~/Documents/SimpleIDE/Learn/Simple\ Libraries/Robotics/Arlo/libarlodrive/arlodrive.c ]]; then
  if ! [[ -d ~/Documents/SimpleIDE/ ]]; then
    mkdir -p ~/Documents/SimpleIDE/
  fi
  cd ~/Documents/SimpleIDE/
  wget https://www.parallax.com/sites/default/files/downloads/Learn-Folder-Updated-2019.07.02_0.zip
  unzip Learn-Folder-Updated-2019.07.02_0.zip
  cd
fi

# We will use ~/.arlobot to store "private" data
# That is data that doesn't need to be part of
# the public github repo like user tokens,
# sounds, and room maps and per robot settings
if ! [[ -d ${HOME}/.arlobot ]]; then
  mkdir "${HOME}/.arlobot"
fi

ARLO_HOME=${HOME}/.arlobot

if [[ -e ${ARLO_HOME}/arlobot.yaml ]]; then
  if ! (diff "${HOME}/catkin_ws/src/ArloBot/src/arlobot/arlobot_bringup/param/arlobot.yaml" "${ARLO_HOME}/arlobot.yaml" >/dev/null); then
    printf "\n${GREEN}The ${RED}arlobot.yaml${GREEN} file in the repository is different from the one${NC}\n"
    printf "${GREEN}in your local settings.${NC}\n"
    printf "${GREEN}This is expected, but just in case, please look over the differences,${NC}\n"
    printf "${GREEN}and see if you need to copy in any new settings, or overwrite the file completely:${NC}\n"
    diff "${HOME}/catkin_ws/src/ArloBot/src/arlobot/arlobot_bringup/param/arlobot.yaml" "${ARLO_HOME}/arlobot.yaml" || true
    cp -i "${HOME}/catkin_ws/src/ArloBot/src/arlobot/arlobot_bringup/param/arlobot.yaml" "${ARLO_HOME}/"
    printf "\n"
  fi
else
  printf "\n"
  cp "${HOME}/catkin_ws/src/ArloBot/src/arlobot/arlobot_bringup/param/arlobot.yaml" "${ARLO_HOME}/"
  printf "${GREEN}A brand new ${RED}${ARLO_HOME}/arlobot.yaml${GREEN} file has been created,${NC}\n"
  printf "${LIGHT_PURPLE}Please edit this file to customize according to your robot!${NC}\n"
fi

for i in "${HOME}/catkin_ws/src/ArloBot/PropellerCodeForArloBot/dotfiles/"*; do
  [[ -e "${i}" ]] || break # handle the case of no files
  # https://stackoverflow.com/a/9011264/4982408
  if [[ -e ${ARLO_HOME}/${i##*/} ]]; then
    if ! (diff "${i}" "${ARLO_HOME}/${i##*/}" >/dev/null); then
      printf "\n${GREEN}The ${RED}${i##*/}${GREEN} file in the repository is different from the one${NC}\n"
      printf "${GREEN}in your local settings.${NC}\n"
      printf "${GREEN}This is expected, but just in case, please look over the differences,${NC}\n"
      printf "${GREEN}and see if you need to copy in any new settings, or overwrite the file completely:${NC}\n"
      diff "${i}" "${ARLO_HOME}/${i##*/}" || true
      cp -i "${i}" "${ARLO_HOME}/"
    fi
  else
    printf "\n"
    cp "${i}" "${ARLO_HOME}/"
    printf "${GREEN}A brand new ${RED}${ARLO_HOME}/${i##*/}${GREEN} file has been created,${NC}\n"
    printf "${LIGHT_PURPLE}Please edit this file to customize according to your robot!${NC}\n"
  fi
done

if [[ ! -e /etc/cron.d/arlobot ]]; then
  printf "\n${YELLOW}[Adding cron job to start web server on system reboot.]${NC}\n"
  echo "@reboot $(whoami) ${HOME}/catkin_ws/src/ArloBot/startpm2.sh > ${HOME}/crontab.log" | sudo tee -a /etc/cron.d/arlobot >/dev/null
fi

printf "\n${LIGHTPURPLE}[Flushing PM2 logs and starting/restarting web server.]${NC}\n"
pm2 flush
if ! pm2 restart Robot; then
  "${HOME}/catkin_ws/src/ArloBot/startpm2.sh"
fi

if [[ "${USER}" == chrisl8 ]]; then
  # NOTE: It is OK if this section "crashes" the script on a failure,
  # because it ONLY runs for me, not end users.
  if ! [[ -d /home/robotStatusUser ]]; then
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
  #if ! (command -v aoss > /dev/null)
  #    then
  #    printf "\n${YELLOW}[Adding aoss for Text To Speech.]${NC}\n"
  #    sudo apt install -y alsa-oss
  #    printf "\n${GREEN}Don't for get to insatll Cepstral Voice!${NC}\n"
  #fi

  # Special notices for the developer himself to keep his stuff up to date. =)
  cd "${HOME}/catkin_ws/src/ArloBot/node"
  printf "\n${RED}[Hey ${USER} please make sure the below items are up to date!]${NC}\n"
  printf "\n${GREEN}[Hey ${USER} please make sure the below items are up to date!]${NC}\n"
  printf "\n${PURPLE}[Hey ${USER} please make sure the below items are up to date!]${NC}\n"
  printf "${YELLOW}Does the current version of nvm we installed:${NC} "
  nvm --version
  printf "${YELLOW}Match the version on github:${NC} "
  wget -qO- https://github.com/creationix/nvm/blob/master/README.md | grep install.sh | grep wget | sed -e "s/<pre><code>//" | sed "s/\//\\n/g" | grep ^v | head -1
  printf "\n${YELLOW}You are using this version of node:${NC} "
  node --version
  printf "${YELLOW}and this is the current stable version of node:${NC} "
  wget -qO- https://nodejs.org/en/download/ | grep "Latest LTS Version:" | sed "s/<\/p>//g" | sed "s/.*<strong>//" | sed "s/<.*//"
  printf "\n${YELLOW}Checking for out of date global node modules:${NC}\n"
  npm outdated -g || true # Log.io will always be "old", so do not let failures crash the script.
  printf "${YELLOW}Checking for out of date package node modules:${NC}\n"
  printf "${YELLOW}in node/:${NC}\n"
  npm outdated || true # Informational, do not crash  script
  printf "${YELLOW}in website/:${NC}\n"
  cd "${HOME}/catkin_ws/src/ArloBot/website"
  npm outdated || true # Informational, do not crash  script
  printf "${PURPLE}-------------------------------------------------------${NC}\n"
fi

printf "\n${PURPLE}Anytime you want to update ArloBot code from the web you can run this same script again. It will pull down and compile new code without wiping out custom configs in ~/.arlarbot. I run this script myself almost every day.${NC}\n"

printf "\n${YELLOW}-----------------------------------${NC}\n"
printf "${YELLOW}ALL DONE! REBOOT AND START TESTING!${NC}\n"
printf "\n"
printf "${LIGHTCYAN}Go to ${LIGHTBLUE}http://$(node "${HOME}/catkin_ws/src/ArloBot/node/ipAddress.js"):$(jq '.webServerPort' "${HOME}/.arlobot/personalDataForBehavior.json")${LIGHTCYAN} to see the Arlobot web interface.${NC}\n"
printf "\n"
# TODO: Find another pretty color and remind about editting files.
printf "${BLUE}I have a list of tests here: cat ${HOME}/catkin_ws/src/ArloBot/manualTests.txt${NC}\n"
printf "${GREEN}Look at README.md for testing ideas.${NC}\n"
printf "${GREEN}See here for your next step: ${BLUE}http://ekpyroticfrood.net/?p=165\n${NC}\n"

printf "\n${YELLOW}------------------------------------------------------------${NC}\n"
printf "${YELLOW}Remember: You MUST install the Propeller code on your Propeller board too!${NC}\n"
printf "${GREEN}See: ${BLUE}https://ekpyroticfrood.net/?p=165${NC}\n"
printf "${GREEN}for more information on getting SimpleIDE set up.${NC}\n"
printf "${YELLOW}------------------------------------------------------------${NC}\n"
INSTALL_FINISHED="true"
