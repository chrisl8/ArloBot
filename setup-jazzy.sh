#!/usr/bin/env bash
# shellcheck disable=SC2059 disable=SC2129
# ROS Arlobot Automated Install

INSTALLING_ROS_DISTRO=jazzy

# Run this straight off of github like this:
# bash <(wget -qO- --no-cache -o /dev/null https://raw.githubusercontent.com/chrisl8/ArloBot/jazzy/setup-jazzy.sh)

#   TESTING
#
# Testing install with Docker:
#
# You can Test this with Docker by installing Docker, then pulling down the Ubuntu 24.04 image:
# docker pull ubuntu:24.04
# cd ${HOME}/ArloBot # Or wherever you cloned the code to.
#
# Then either kick it off all in one shot:
# docker run -ti -v $PWD:/root/ArloBot ubuntu:24.04 /bin/bash -c "~/ArloBot/setup-jazzy.sh"
#
# Or start an interactive shell in Docker and run it, with the ability to make changes and start it again when it finishes:
# docker run -ti -v $PWD:/root/ArloBot ubuntu:24.04 /bin/bash
# ~/ArloBot/setup-jazzy.sh
#
# If you started a non-interactive ("one shot") build and then it crashed and you want to get in and look around:
# https://docs.docker.com/engine/reference/commandline/commit/
# Find the name of the container:
# docker ps -a
# docker commit $CONTAINER_ID mytestimage
# docker run -ti -v $PWD:/home/user mytestimage /bin/bash
#
# and when you are done delete the image:
# docker image rm mytestimage
#
# Then you can look around and try running the script if you want again.
#
#
# To clean up Docker when you are done run:
# docker system prune
#
# Also note that if you add --rm to the run command on any docker command above, it will automatically remove the container
# after you leave it, instead of leaving it hanging around.

set -e

#BLUE='\033[0;34m' # This is almost impossible to read on a black background!
GREEN='\033[0;32m'
RED='\033[0;31m'
PURPLE='\033[0;35m'
LIGHT_PURPLE='\033[1;35m'
YELLOW='\033[1;33m'
LIGHTCYAN='\033[1;36m'
LIGHTBLUE='\033[1;34m'
NC='\033[0m' # NoColor

ARLO_HOME=${HOME}/.arlobot

function finish() {
  if [[ -z ${INSTALL_FINISHED} ]]; then
    printf "\n"
    printf "${RED}INSTALL FAILURE!!!${NC}\n"
    printf "${RED}The Install Script has failed. Please investigate the cause, correct, and run again before proceeding.${NC}\n"
    printf "\n"
    printf "${YELLOW}If this was a transient error, such as a network failure connecting to something, you may just need to run it again.${NC}\n"
    printf "\n"
    printf "${YELLOW}If this persists, please file an issue against the repository at https://github.com/chrisl8/ArloBot/issues${NC}\n"
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
  printf "${LIGHTBLUE}Adding settings and basic packages for Docker based Ubuntu images.${NC}\n"
  # The docker image has no /etc/localtime
  # When the prereq install installs the tzdat package,
  # It stops and asks for time zone info.
  # This should prevent that.
  # https://bugs.launchpad.net/ubuntu/+source/tzdata/+bug/1773687
  export DEBIAN_FRONTEND=noninteractive
  # This won't work inside of sudo though, so just install tzdata now
  # rather than letting it get picked up later as a pre-req,
  # and add the other things we know Docker is missing too while we are at it.
  apt update # Docker doesn't have the latest package lists by default.
  apt install -y sudo python3
  # Now the rest of the script should work as if it was in a normal Ubuntu install.

  # Installing this now appears to prevent it from hanging up the Docker setup later.
  apt install -y keyboard-configuration

  DOCKER_TEST_INSTALL=true
fi

printf "${YELLOW}[Updating local pacakge list.]${NC}\n"
sudo apt update

if ! (command -v lsb_release >/dev/null); then
  sudo apt install -y lsb-release
fi
version=$(lsb_release -sc)

printf "\n${YELLOW}[Checking the Ubuntu version]${NC}\n"
printf "${LIGHTBLUE}Ubuntu ${version} found${NC}\n"
case ${version} in
"noble") ;;

*)
  printf "${RED}[This script will only work on Ubuntu Noble (24.04)]${NC}\n"
  exit 1
  ;;
esac

if ! (command -v python >/dev/null); then
  printf "\n${YELLOW}[Setting Python 3 as System Default]${NC}\n"
  printf "${LIGHTBLUE}Because currently there IS NO python otherwise!${NC}\n"
  sudo update-alternatives --install /usr/bin/python python /usr/bin/python3 10
fi

if ! [[ ${DOCKER_TEST_INSTALL=true} == "true" ]]; then # This does not work on Docker
  printf "\n${YELLOW}[Updating Root CA Certificates from Ubuntu]${NC}\n"
  # Sometimes this has to be done by hand on new Ubuntu installs.
  sudo update-ca-certificates
fi

if ! (locale | grep "en_US.UTF-8" >/dev/null); then
  printf "\n${YELLOW}[Setting locale to en_US.UTF-8 as recommended by Jazzy]${NC}\n"
  if ! (command -v locale-gen >/dev/null); then
    sudo apt install -y locales
  fi
  sudo locale-gen en_US en_US.UTF-8
  sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
  export LANG=en_US.UTF-8
fi

printf "\n${YELLOW}[Ensuring Ubuntu Universe repository is enabled.]${NC}\n"
sudo apt install -y software-properties-common
sudo add-apt-repository -y universe

if ! [[ -e /etc/apt/sources.list.d/ros2.list ]]; then
  printf "${YELLOW}[Adding the ROS repository]${NC}\n"
  # This should follow the official ROS install instructions closely.
  #      https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html
  # That is why there is a separate section for extra packages that I need for Arlo.
  if ! (command -v gpg >/dev/null); then
    sudo apt install -y gnupg
  fi
  if ! (command -v gpg2 >/dev/null); then
    sudo apt install -y gnupg2
  fi
  if ! (command -v curl >/dev/null); then
    sudo apt install -y curl
  fi

  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
  sudo sh -c "echo \"deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main\" > /etc/apt/sources.list.d/ros2.list"

fi

printf "\n${YELLOW}[Updating & upgrading all existing Ubuntu packages]${NC}\n"
sudo apt update
sudo apt upgrade -y
sudo apt autoremove -y

# This should follow the official ROS install instructions closely.
#      http://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
# That is why there is a separate section for extra packages that I need for Arlo.
# Note, while we are NOT building ROS from source, we are building SOME THINGS from source,
# and hence must also follow the the instructions to set up a build environment.
if ! (dpkg -s ros-${INSTALLING_ROS_DISTRO}-desktop | grep "Status: install ok installed" &>/dev/null) || ! (command -v rosdep); then
  printf "\n${YELLOW}[Installing ROS]${NC}\n"
  sudo apt install -y ros-${INSTALLING_ROS_DISTRO}-desktop
  printf "${LIGHTBLUE}ROS installed!${NC}\n"
  printf "\n${YELLOW}[Installing ROS Dependencies for building packages]${NC}\n"
  sudo apt install -y python3-rosdep python3-colcon-common-extensions build-essential
  if ! [[ -e /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    printf "\n${LIGHTBLUE}Running rosdep init . . . ${NC}\n"
    sudo sh -c "rosdep init"
  fi
  printf "${LIGHTBLUE}Running rosdep update . . .${NC}\n"
  rosdep update
  # END Official ROS Install section
fi

# shellcheck source=/opt/ros/jazzy/setup.bash
source /opt/ros/${INSTALLING_ROS_DISTRO}/setup.bash

if [[ -z ${USER} ]]; then
  # This probably only happens in Docker, but note that it isn't just for my own purpose
  # that I need $USER, npm uses it too, else you get weird errors like:
  #sh: 1: cannot create build_log.txt: Permission denied
  # even when running as root! if $USER is not set
  USER=$(whoami)
  export USER
fi

if [[ ! -d ${ARLO_HOME} ]]; then
  printf "\n${YELLOW}[Creating .arlobot folder]${NC}\n"
  printf "${GREEN}This holds personal data for your robot.${NC}\n"
  # We will use ${HOME}/.arlobot to store "private" data
  # That is data that doesn't need to be part of
  # the public github repo like user tokens,
  # sounds, and room maps and per robot settings
  mkdir "${ARLO_HOME}"
fi

# Collect and save responses about what to install or not

printf "\n${YELLOW}[Installing additional Ubuntu and ROS Packages for Arlo]${NC}\n"
printf "${LIGHTBLUE}This runs every time, in case new packages were added.${NC}\n"

PACKAGE_TO_INSTALL_LIST=()
# ### Required Packages and Why ###
PACKAGE_TO_INSTALL_LIST+=(jq)
# jq - allows shell scripts to read .json formatted config files.
# jq is used to read and write JSON configuration files in shell scripts
# for the installer script, and for many Arlobot functions.
PACKAGE_TO_INSTALL_LIST+=(python3-argcomplete)
# python3-argcomplete - The Jazzy Install page recommends installing this.
PACKAGE_TO_INSTALL_LIST+=(git)
# git - allows for cloning of repositories
PACKAGE_TO_INSTALL_LIST+=(vim)
# vim - We aren't going to get very far without being able to edit some files.
PACKAGE_TO_INSTALL_LIST+=(xvfb)
# xvfb is required for Cypress testing to work.
PACKAGE_TO_INSTALL_LIST+=(cron)
# cron - required for running scheduled tasks
PACKAGE_TO_INSTALL_LIST+=(speech-dispatcher)
# speech-dispatcher contains spd-say which is used by some scripts to speak
PACKAGE_TO_INSTALL_LIST+=(moreutils)
# moreutils - sponge is used by some of my scripts
PACKAGE_TO_INSTALL_LIST+=(python3-pip)
# python3-pip - Required to install Python tools for things such as
#      USB Relay reader.
PACKAGE_TO_INSTALL_LIST+=(openssh-server)
# openssh-server - required to SSH into robot remotely
PACKAGE_TO_INSTALL_LIST+=(python3-serial)
# python3-serial - required for ROS to talk to the Propeller Activity Board
PACKAGE_TO_INSTALL_LIST+=(expect)
# expect - required to get 'unbuffer' which is required by node to spawn ROS commands and get real time stdout data
# http://stackoverflow.com/a/11337310
# http://linux.die.net/man/1/unbuffer
PACKAGE_TO_INSTALL_LIST+=(imagemagick)
# imagemagick - used to grab screen shots of desktop for web local robot website display
PACKAGE_TO_INSTALL_LIST+=(fswebcam)
# fswebcam - Used for streaming a camera to the website. Camera streaming will not work without it.
PACKAGE_TO_INSTALL_LIST+=(zbar-tools)
# zbar-tools - Used by node service to read QR codes via on board camera
PACKAGE_TO_INSTALL_LIST+=(libftdi1-dev)
# libftdi1-dev - required by pylibftdi for talking to USB based serial boards like relay boards, etc.
#           https://pylibftdi.readthedocs.io/en/0.18.0/installation.html
#      For 8-CH USB Relay board:
#           Reference: https://code.google.com/p/drcontrol/wiki/Install_RaspberryPi">https://code.google.com/p/drcontrol/wiki/Install_RaspberryPi
#           TEST:
#           python -m pylibftdi.examples.list_devices
#           Should return:
#           FTDI:FT245R USB FIFO:A9026EI5
#           If you have a USB Relay board attached via USB.

sudo apt install -y "${PACKAGE_TO_INSTALL_LIST[@]}"

ROS2_WS=ros2_ws

if ! [[ -d ${HOME}/${ROS2_WS}/src ]]; then
  printf "\n${YELLOW}[Creating the ROS Development workspace and testing with colcon]${NC}\n"
  mkdir -p "${HOME}/${ROS2_WS}/src"
  cd "${HOME}/${ROS2_WS}"
  # https://robotics.stackexchange.com/a/113133
  PYTHONWARNINGS="ignore:easy_install command is deprecated,ignore:setup.py install is deprecated"
  export PYTHONWARNINGS
  colcon build --symlink-install
  # shellcheck source=/home/chrisl8/${ROS2_WS}/install/setup.bash
  source "${HOME}/${ROS2_WS}/install/setup.bash"
fi

printf "\n${YELLOW}[Cloning or Updating git repositories]${NC}\n"

if ! [[ ${DOCKER_TEST_INSTALL=true} == "true" ]]; then # This does not work on Docker
  printf "${LIGHTBLUE}ArloBot repository${NC}\n"
  if ! [[ -d "${HOME}/ArloBot" ]]; then
    cd "${HOME}"
    git clone -b jazzy https://github.com/chrisl8/ArloBot.git
  else
    cd "${HOME}/ArloBot"
    git fetch
    git checkout jazzy
    git pull
  fi
fi

if ! [[ -e ${HOME}/${ROS2_WS}/src/arlobot_ros ]]; then
  ln -s "${HOME}/ArloBot/arlobot_ros" "${HOME}/${ROS2_WS}/src/"
fi

if ! [[ -e ${HOME}/${ROS2_WS}/src/arlobot_interfaces ]]; then
  ln -s "${HOME}/ArloBot/arlobot_interfaces" "${HOME}/${ROS2_WS}/src/"
fi

printf "\n${LIGHTBLUE}Slamtec RPLIDAR${NC}\n"
cd "${HOME}/${ROS2_WS}/src"
if ! [[ -d ${HOME}/rplidar_ros ]]; then
  cd "${HOME}"
  git clone -b ros2 https://github.com/Slamtec/rplidar_ros.git
else
  cd "${HOME}/rplidar_ros"
  git fetch
  git checkout ros2
  git pull
fi

if ! [[ -e ${HOME}/${ROS2_WS}/src/rplidar_ros ]]; then
  ln -s "${HOME}/rplidar_ros" "${HOME}/${ROS2_WS}/src/rplidar_ros"
fi

export PIP_DISABLE_PIP_VERSION_CHECK=1

printf "\n${YELLOW}[Installing dependencies for ROS build-from-source packages.]${NC}\n"
cd "${HOME}/${ROS2_WS}"
printf "${LIGHTBLUE}Running rosdep update . . .${NC}\n"
rosdep update
rosdep install -q -y -r --from-paths src --ignore-src --rosdistro ${INSTALLING_ROS_DISTRO}

printf "\n${YELLOW}[Generating URDF file with xacro.]${NC}\n"
/opt/ros/jazzy/bin/xacro "${HOME}/ArloBot/urdf-xacro-source/common.urdf.xacro" > "${HOME}/ArloBot/arlobot_ros/arlobot.urdf"

printf "\n${YELLOW}[(Re)Building ROS Source files.]${NC}\n"
cd "${HOME}/${ROS2_WS}"
# https://robotics.stackexchange.com/a/113133
PYTHONWARNINGS="ignore:easy_install command is deprecated,ignore:setup.py install is deprecated"
export PYTHONWARNINGS
colcon build --symlink-install
# shellcheck source=/home/chrisl8/${ROS2_WS}/install/setup.bash
source "${HOME}/${ROS2_WS}/install/setup.bash"

if [[ -f ${HOME}/.bashrc ]]; then
  if ! (grep "${HOME}/ArloBot/scripts" "${HOME}/.bashrc" >/dev/null); then
    printf "\n${YELLOW}[Adding ArloBot Scripts folder to your path in .bashrc]${NC}\n"
    sh -c "echo \"export PATH=\\\$PATH:${HOME}/ArloBot/scripts\" >> ${HOME}/.bashrc"
  fi
fi

if [[ -f ${HOME}/.zshrc ]]; then
  if ! (grep "${HOME}/ArloBot/scripts" "${HOME}/.zshrc" >/dev/null); then
    printf "\n${YELLOW}[Adding ArloBot Scripts folder to your path in .zshrc]${NC}\n"
    sh -c "echo \"export PATH=\\\$PATH:${HOME}/ArloBot/scripts\" >> ${HOME}/.zshrc"
  fi
fi

# For 8-CH USB Relay board:
pip3 install pylibftdi --break-system-packages
# Required by pylibftdi
# https://pylibftdi.readthedocs.io/en/0.15.0/installation.html
if ! [[ -f /etc/udev/rules.d/99-libftdi.rules ]]; then
  printf "\n${YELLOW}[Adding required sudo rule for pylibftdi to access USB based serial ports.]${NC}\n"
  sudo "${HOME}/ArloBot/scripts/addRuleForUSBRelayBoard.sh"
  printf "${RED}You may have to reboot before the USB Relay board will function!${NC}\n"
fi

export NVM_SYMLINK_CURRENT=true
NVM_TAG=$(curl -s curl -s https://api.github.com/repos/nvm-sh/nvm/releases/latest | grep tag_name | cut -d '"' -f 4 | cut -d 'v' -f 2)
NVM_VERSION_LATEST="${NVM_TAG//v/}"
NVM_VERSION=None

if [[ -e ${HOME}/.nvm/nvm.sh ]]; then
  export NVM_DIR="$HOME/.nvm"
  # shellcheck source=/home/chrisl8/.nvm/nvm.sh
  [[ -s "$NVM_DIR/nvm.sh" ]] && . "$NVM_DIR/nvm.sh" # This loads nvm
fi
if [[ -e ${HOME}/.config/nvm/nvm.sh ]]; then
  export NVM_DIR="$HOME/.config/nvm"
  # shellcheck source=/home/chrisl8/.nvm/nvm.sh
  [[ -s "$NVM_DIR/nvm.sh" ]] && . "$NVM_DIR/nvm.sh" # This loads nvm
fi

if (command -v nvm); then
  NVM_VERSION=$(nvm --version)
fi

if [ "$NVM_VERSION" != "$NVM_VERSION_LATEST" ]; then
  printf "\n${BRIGHT_MAGENTA}Updating NVM from ${NVM_VERSION} to ${NVM_VERSION_LATEST}${NC}\n"

  if [[ -e ${HOME}/.nvm/nvm.sh ]]; then
    printf "${LIGHTBLUE}Deactivating existing Node Version Manager:${NC}\n"
    # shellcheck source=/home/chrisl8/.nvm/nvm.sh
    [[ -s "$NVM_DIR/nvm.sh" ]] && . "$NVM_DIR/nvm.sh" # This loads nvm
    nvm deactivate
  fi

  wget -qO- "https://raw.githubusercontent.com/nvm-sh/nvm/v$NVM_TAG/install.sh" | bash

  if [[ -e ${HOME}/.bashrc ]] && ! (grep NVM_SYMLINK_CURRENT "${HOME}/.bashrc" >/dev/null); then
    printf "\n${YELLOW}[Setting the NVM current environment in your .bashrc file]${NC}\n"
    sh -c "echo \"export NVM_SYMLINK_CURRENT=true\" >> ${HOME}/.bashrc"
  fi
  if  [[ -e ${HOME}/.zshrc ]] && ! (grep NVM_SYMLINK_CURRENT "${HOME}/.zshrc" >/dev/null); then
    printf "\n${YELLOW}[Setting the NVM current environment in your .zshrc file]${NC}\n"
    sh -c "echo \"export NVM_SYMLINK_CURRENT=true\" >> ${HOME}/.zshrc"
  fi
fi

printf "\n${YELLOW}[Installing and Activating the Latest Node LTS version]${NC}\n"
if [[ -e ${HOME}/.nvm/nvm.sh ]]; then
  export NVM_DIR="$HOME/.nvm"
fi
if [[ -e ${HOME}/.config/nvm/nvm.sh ]]; then
  export NVM_DIR="$HOME/.config/nvm"
fi
# shellcheck source=/home/chrisl8/.nvm/nvm.sh
[[ -s "$NVM_DIR/nvm.sh" ]] && . "$NVM_DIR/nvm.sh" # This loads nvm
nvm install --lts
nvm alias default "lts/*"

printf "\n${YELLOW}[Grabbing/Updating global dependencies for node packages]${NC}\n"
printf "${LIGHTBLUE}You may get some errors here, that is normal. As long as things work, it is OK.$NC\n"
cd
printf "\n${YELLOW}[PM2 for running Robot service]$NC\n"
npm install -g pm2
printf "\n${YELLOW}[Log Streamer for Website]$NC\n"
npm install -g log.io
printf "\n${YELLOW}[Log.io File Watcher for Log.io Log Streamer]$NC\n"
npm install -g log.io-file-input

if [[ -d ${HOME}/ArloBot/Log.io ]]; then
  printf "\n"
  printf "${LIGHTBLUE}Removing old Log.io Install${NC}\n"
  rm -rf "${HOME}/ArloBot/Log.io"
fi

cd "${HOME}/ArloBot/node"
printf "\n${YELLOW}[Grabbing node dependencies for scripts]${NC}\n"
printf "${LIGHTBLUE}You may get some errors here, that is normal. As long as things work, it is OK.$NC\n"
npm update

cd "${HOME}/ArloBot/website"
printf "\n${YELLOW}[Grabbing node dependencies for React website]${NC}\n"
npm update
printf "\n${YELLOW}[Building React website]${NC}\n"
npm run build

cd "${HOME}/ArloBot/cypress-tests"
printf "\n${YELLOW}[Installing Cypress.io for Tests]$NC\n"
npm update

if [[ -e ${ARLO_HOME}/personalDataForBehavior.json ]]; then
  node "${HOME}/ArloBot/node/personalData.js"
else
  printf "\n"
  cp "${HOME}/ArloBot/scripts/dotarlobot/personalDataForBehavior.json" "${ARLO_HOME}/"
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

if ! (id | grep dialout >/dev/null); then
  printf "\n${GREEN}Adding your user to the 'dialout' group for serial port access.${NC}\n"
  sudo adduser "${USER}" dialout >/dev/null
  printf "${RED}You may have to reboot before you can use the Propeller Activity Board.${NC}\n"
fi

if ! (id | grep video >/dev/null); then
  printf "\n${GREEN}Adding your user to the 'video' group for access to cameras.${NC}\n"
  sudo adduser "${USER}" video >/dev/null
fi

if ! (sudo -nl | grep resetUSB >/dev/null && sudo -nl | grep modprobe >/dev/null); then
  printf "\n${YELLOW}[Setting up required sudo entries.]${NC}\n"
  echo "${USER} ALL = NOPASSWD: ${SCRIPTDIR}/resetUSB.sh" >/tmp/arlobot_sudoers
  echo "${USER} ALL = NOPASSWD: /sbin/modprobe" >>/tmp/arlobot_sudoers
  chmod 0440 /tmp/arlobot_sudoers
  sudo chown root:root /tmp/arlobot_sudoers
  sudo mv /tmp/arlobot_sudoers /etc/sudoers.d/
  sudo chown root:root /etc/sudoers.d/arlobot_sudoers
fi

if ! (crontab -l >/dev/null 2>&1) || ! (crontab -l | grep startpm2 >/dev/null 2>&1); then
  printf "\n${YELLOW}[Adding cron job to start web server on system reboot.]${NC}\n"
  # https://stackoverflow.com/questions/4880290/how-do-i-create-a-crontab-through-a-script
  (
    echo "@reboot ${HOME}/ArloBot/startpm2.sh > ${HOME}/crontab.log"
  ) | crontab -
fi

printf "\n${LIGHT_PURPLE}[Flushing PM2 logs and starting/restarting web server.]${NC}\n"
pm2 update
pm2 flush
if ! pm2 restart Robot; then
  "${HOME}/ArloBot/startpm2.sh"
fi

if [[ "${USER}" == chrisl8 ]]; then
  # NOTE: It is OK if this section "crashes" the script on a failure,
  # because it ONLY runs for me, not end users.
  if ! [[ -d /home/robotStatusUser ]]; then
    printf "\n${YELLOW}[Adding robotStatusUser.]${NC}\n"
    printf "${GREEN}(This is NOT required for Arlobot, just a personal thing.)${NC}\n"
    sudo useradd -m robotStatusUser
    printf "${GREEN}Be sure to add your key to ${HOME}robotStatusUser/.ssh/authorized_keys${NC}\n"
    printf "${GREEN}for anyone who needs to use it!${NC}\n"
    printf "${RED}sudo su - robotStatusUser${NC}\n"
    printf "${RED}mkdir .ssh${NC}\n"
    printf "${RED}vim .ssh/authorized_keys${NC}\n"
    printf "${GREEN}(This is NOT required for Arlobot, just a personal thing.)${NC}\n"
  fi

  # Special notices for the developer himself to keep his stuff up to date. =)
  cd "${HOME}/ArloBot/node"
  printf "\n${RED}[Hey ${USER} please make sure the below items are up to date!]${NC}\n"
  printf "\n${GREEN}[Hey ${USER} please make sure the below items are up to date!]${NC}\n"
  printf "\n${PURPLE}[Hey ${USER} please make sure the below items are up to date!]${NC}\n"
  printf "${YELLOW}Does the current version of nvm we installed:${NC} "
  nvm --version
  printf "${YELLOW}Match the version on github:${NC} "
  curl -s https://api.github.com/repositories/612230/releases/latest | grep tag_name | cut -d '"' -f 4
  printf "\n${YELLOW}You are using this version of node:${NC} "
  node --version
  printf "${YELLOW}and this is the current stable version of node:${NC} "
  wget -qO- https://nodejs.org/en/download/ | grep "Latest LTS Version:" | sed "s/<\/p>//g" | sed "s/.*<strong>//" | sed "s/<.*//"
  printf "\n${YELLOW}Checking for out of date global node modules:${NC}\n"
  npm outdated -g || true # Informational, do not crash  script
  printf "${YELLOW}Checking for out of date package node modules:${NC}\n"
  printf "${YELLOW}in node/:${NC}\n"
  npm outdated || true # Informational, do not crash  script
  printf "${YELLOW}in website/:${NC}\n"
  cd "${HOME}/ArloBot/website"
  npm outdated || true # Informational, do not crash  script
  printf "${YELLOW}in cypress tests/:${NC}\n"
  cd "${HOME}/ArloBot/cypress-tests"
  npm outdated || true # Informational, do not crash  script
  printf "${PURPLE}-------------------------------------------------------${NC}\n"

  printf "${YELLOW}Diffing your param files to defaults where available:${NC}\n"
  printf "${LIGHTBLUE}teb_local_planner_params:${NC}\n"
  diff "${HOME}/${ROS2_WS}/src/teb_local_planner_tutorials/cfg/diff_drive/teb_local_planner_params.yaml" "${HOME}/${ROS2_WS}/src/arlobot_ros/param/teb_local_planner_params.yaml" || true
  printf "${LIGHTBLUE}twist_mux_locks:${NC}\n"
  diff /opt/ros/${INSTALLING_ROS_DISTRO}/share/twist_mux/config/twist_mux_locks.yaml "${HOME}/${ROS2_WS}/src/arlobot_ros/param/twist_mux_locks.yaml" || true
  printf "${LIGHTBLUE}twist_mux_topics:${NC}\n"
  diff /opt/ros/${INSTALLING_ROS_DISTRO}/share/twist_mux/config/twist_mux_topics.yaml "${HOME}/${ROS2_WS}/src/arlobot_ros/param/twist_mux_topics.yaml" || true
  printf "${LIGHTBLUE}mapper_params_localization:${NC}\n"
  diff "${HOME}/${ROS2_WS}/src/slam_toolbox/slam_toolbox/config/mapper_params_localization.yaml" "${HOME}/${ROS2_WS}/src/arlobot_ros/param/mapper_params_localization.yaml" || true
  printf "${LIGHTBLUE}mapper_params_online_async:${NC}\n"
  diff "${HOME}/${ROS2_WS}/src/slam_toolbox/slam_toolbox/config/mapper_params_online_async.yaml" "${HOME}/${ROS2_WS}/src/arlobot_ros/param/mapper_params_online_async.yaml" || true
fi

printf "\n${PURPLE}Anytime you want to update ArloBot code from the web you can run this same script again. It will pull down and compile new code without wiping out custom configs in ${HOME}/.arlobot. I run this script myself almost every day.${NC}\n"

printf "\n${YELLOW}-----------------------------------${NC}\n"
printf "${YELLOW}ALL DONE! REBOOT, EDIT FILES, AND START TESTING!${NC}\n\n"
printf "${GREEN}Remember to edit the config files in ${HOME}/.arlobot${NC}\n\n"
printf "${LIGHTCYAN}Go to ${LIGHTBLUE}http://$(node "${HOME}/ArloBot/node/ipAddress.js"):$(jq '.webServerPort' "${ARLO_HOME}/personalDataForBehavior.json")${LIGHTCYAN} to see the Arlobot web interface.${NC}\n"
printf "\n"
printf "${GREEN}Look at README.md for testing ideas.${NC}\n"

printf "\n${YELLOW}------------------------------------------------------------${NC}\n"
printf "${YELLOW}Remember: You MUST install the Propeller code on your Propeller Activity Board too!${NC}\n"
printf "\n${YELLOW}The Propeller code cannot be built on a Pi! It has to be done on an x86_64 system!${NC}\n"
printf "${LIGHTBLUE}You will need to build this code on an x86_64 system and  load the code onto your Propeller Activity Board before this code can work on your robot!${NC}\n"
printf "${LIGHTCYAN}You can run${NC}\n"
printf "${LIGHTCYAN}bash <(wget -qO- --no-cache -o /dev/null https://raw.githubusercontent.com/chrisl8/ArloBot/jazzy/Install_Propeller_Code.sh)${NC}\n"
printf "${LIGHTCYAN}to perform the install,${NC}\n"
printf "${LIGHTCYAN}and then use PropellerSerialTest.sh back on the Pi here to test it.${NC}\n"
printf "${GREEN}See: ${LIGHTBLUE}https://ekpyroticfrood.net/?p=551${NC}\n"
printf "${GREEN}for more information on installing code on your Propeller Activity Board.${NC}\n"
printf "${YELLOW}------------------------------------------------------------${NC}\n"

INSTALL_FINISHED="true"
