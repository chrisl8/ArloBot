#!/usr/bin/env bash
# shellcheck disable=SC2059 disable=SC2129
# ROS Arlobot Automated Install

# TODO: Remove all noetic references.

INSTALLING_ROS_DISTRO=jazzy

# Run this straight off of github like this:
# bash <(wget -qO- --no-cache https://raw.githubusercontent.com/chrisl8/ArloBot/noetic/setup-jazzy.sh)

#   TESTING
#
# Testing install with Docker:
#
# You can Test this with Docker by installing Docker, then pulling down the Ubuntu 20.04 image:
# sudo docker pull ubuntu:20.04
# cd ~/dev_ws/src/ArloBot
#
# Then either kick it off all in one shot:
# sudo docker run -ti -v $PWD:/home/user ubuntu:20.04 /bin/bash -c "/home/user/setup-jazzy.sh"
#
# Or start an interactive shell in Docker and run it, with the ability to make changes and start it again when it finishes:
# sudo docker run -ti -v $PWD:/home/user ubuntu:20.04 /bin/bash
# /home/user/setup-jazzy.sh
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

printf "\n"
printf "${RED}THIS IS NOT MEANT FOR USE!!! It is just me testing.${NC}\n"
printf "This script is part of my personal attempt to make Jazzy work on this robot,\n"
printf "but it doesn't work and possibly never will.\n"
#exit 1

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

printf "${YELLOW}[Updating local pacakge list.]${NC}\n"
sudo apt update

if ! (command -v lsb_release >/dev/null); then
  sudo apt install -y lsb-release
fi
version=$(lsb_release -sc)

printf "\n${YELLOW}[Checking the Ubuntu version]${NC}\n"
printf "${BLUE}Ubuntu ${version} found${NC}\n"
case ${version} in
"noble") ;;

*)
  printf "${RED}[This script will only work on Ubuntu Noble (24.04)]${NC}\n"
  exit 1
  ;;
esac

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
  apt install -y tzdata sudo cron
  # Now the rest of the script should work as if it was in a normal Ubuntu install.

  # Installing this now appears to prevent it from hanging up the Docker setup later.
  apt install -y keyboard-configuration

  DOCKER_TEST_INSTALL=true
fi

if ! (command -v python >/dev/null); then
  printf "\n${YELLOW}[Setting Python 3 as System Default]${NC}\n"
  printf "${BLUE}Because currently there IS NO python otherwise!${NC}\n"
  sudo update-alternatives --install /usr/bin/python python /usr/bin/python3 10
fi

if ! [[ ${DOCKER_TEST_INSTALL=true} == "true" ]]; then # This does not work on Docker
  printf "\n${YELLOW}[Updating Root CA Certificates from Ubuntu]${NC}\n"
  # Sometimes this has to be done by hand on new Ubuntu installs.
  sudo update-ca-certificates
  printf "\n"
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
if ! [[ ${TRAVIS} == "true" ]]; then # Upgrading packages in Travis often fails due to timeouts.
  sudo apt upgrade -y
  sudo apt autoremove -y
fi

# This should follow the official ROS install instructions closely.
#      http://wiki.ros.org/noetic/Installation/Ubuntu
# That is why there is a separate section for extra packages that I need for Arlo.
# Note, while we are NOT building ROS from source, we are building SOME THINGS from source,
# and hence must also follow the the instructions to set up a build environment
#      http://wiki.ros.org/noetic/Installation/Source
if ! (dpkg -s ros-${INSTALLING_ROS_DISTRO}-desktop | grep "Status: install ok installed" &>/dev/null) || ! (command -v rosdep) || ! (command -v rosinstall_generator); then
  printf "\n${YELLOW}[Installing ROS]${NC}\n"
  sudo apt install -y ros-${INSTALLING_ROS_DISTRO}-desktop
  printf "${YELLOW}[ROS installed!]${NC}\n"
  printf "\n${BLUE}Installing ROS Dependencies for building packages${NC}\n"
  sudo apt install -y python3-rosdep python3-colcon-common-extensions build-essential
  if ! [[ -e /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    printf "\n${BLUE}Running rosdep init . . . ${NC}\n"
    sudo sh -c "rosdep init"
  fi
  printf "${BLUE}Running rosdep update . . .${NC}\n"
  rosdep update
  # END Official ROS Install section
fi

# shellcheck source=/opt/ros/noetic/setup.bash
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
  # We will use ~/.arlobot to store "private" data
  # That is data that doesn't need to be part of
  # the public github repo like user tokens,
  # sounds, and room maps and per robot settings
  mkdir "${ARLO_HOME}"
fi

# Collect and save responses about what to install or not

SETUP_RESPONSE_FILE=${ARLO_HOME}/setupResponses.json

if ! (command -v jq >/dev/null); then
  # jq - allows shell scripts to read .json formatted config files.
  # jq is used to read and write JSON configuration files in shell scripts
  # for the installer script, and for many Arlobot functions.
  sudo apt install -y jq
fi

function saveResponseData() {
  jq --arg p "${1}" ".${2} = \$p" <"${SETUP_RESPONSE_FILE}" >"${ARLO_HOME}/setupResponses.temp"
  mv "${ARLO_HOME}/setupResponses.temp" "${SETUP_RESPONSE_FILE}"
}

# Set up file to save responses, so that we do not have to ask on every run.
if ! [[ -e ${SETUP_RESPONSE_FILE} ]]; then
  echo "{}" >"${SETUP_RESPONSE_FILE}"
  printf "\n${YELLOW}You will be asked about whether or not to install several things now.${NC}\n"
  printf "${BLUE}Your answers depend on what hardware you have.${NC}\n"
  printf "${BLUE}It will not hurt to install things you do not need.${NC}\n\n"
  printf "${BLUE}Your responses will be saved and used on future runs of this script.${NC}\n\n"
  printf "${BLUE}If you want to erase your answers and answer again run:${NC}\n"
  printf "${BLUE}rm ${SETUP_RESPONSE_FILE}${NC}\n"
  printf "${BLUE}before running this script.${NC}\n\n"
  if ! [[ ${TRAVIS} == "true" ]]; then # Never ask questions in Travis test environment
    read -n 1 -s -r -p "Press any key to continue"
  fi
fi

# Ask questions
WORKSTATION_INSTALL=$(jq -r '.responseToWorkstationQuery' "${SETUP_RESPONSE_FILE}")
if [[ ${WORKSTATION_INSTALL} == 'null' ]]; then
  printf "\n${YELLOW}Do you want this to be a WORKSTATION ONLY install?${NC}\n"
  printf "${BLUE}A 'workstation' is a system that is NOT your robot,${NC}\n"
  printf "${BLUE}but that you want to develop on and/or run ROS tools remotely${NC}\n\n"
  printf "${BLUE}to operate and debug your robot.${NC}\n\n"
  printf "${BLUE}This includes things like running RVIZ to view/control navigation and map making.${NC}\n\n"
  if ! [[ ${TRAVIS} == "true" ]]; then # Never ask questions in Travis test environment
    read -n 1 -s -r -p "Press 'y' to set as Workstation ONLY install " WORKSTATION_INSTALL
  fi
  printf "\n"
fi
saveResponseData "${WORKSTATION_INSTALL}" 'responseToWorkstationQuery'

if ! [[ ${WORKSTATION_INSTALL} == "y" ]]; then

  RESPONSE_TO_SWEEP_QUERY=$(jq -r '.responseToSweepQuery' "${SETUP_RESPONSE_FILE}")
  if [[ ${RESPONSE_TO_SWEEP_QUERY} == 'null' ]]; then
    printf "\n${YELLOW}Do you want to install code for Scanse Sweep?${NC}\n"
    printf "${BLUE}The Scanse Sweep is a rotating laser scanner.${NC}\n"
    printf "${BLUE}It is no longer available.${NC}\n\n"
    printf "${BLUE}https://scanse.io/home/${NC}\n\n"
    if ! [[ ${TRAVIS} == "true" ]]; then # Never ask questions in Travis test environment
      read -n 1 -s -r -p "Press 'y' to install Scanse Sweep code " RESPONSE_TO_SWEEP_QUERY
    fi
    printf "\n"
  fi
  saveResponseData "${RESPONSE_TO_SWEEP_QUERY}" 'responseToSweepQuery'

  RESPONSE_TO_XV11_QUERY=$(jq -r '.responseToXv11Query' "${SETUP_RESPONSE_FILE}")
  if [[ ${RESPONSE_TO_XV11_QUERY} == 'null' ]]; then
    printf "\n${YELLOW}Do you want to install code for Neato XV11?${NC}\n"
    printf "${BLUE}The XV11 was a rotating laser scanner pulled from old vacuum cleaners.${NC}\n"
    printf "${BLUE}If you have one you will need this code.${NC}\n\n"
    if ! [[ ${TRAVIS} == "true" ]]; then # Never ask questions in Travis test environment
      read -n 1 -s -r -p "Press 'y' to install Neato XV11 code " RESPONSE_TO_XV11_QUERY
    fi
    printf "\n"
  fi
  saveResponseData "${RESPONSE_TO_XV11_QUERY}" 'responseToXv11Query'

  RESPONSE_TO_RPLIDAR_QUERY=$(jq -r '.responseToRplidarQuery' "${SETUP_RESPONSE_FILE}")
  if [[ ${RESPONSE_TO_RPLIDAR_QUERY} == 'null' ]]; then
    printf "\n${YELLOW}Do you want to install code for Slamtec RPLIDAR?${NC}\n"
    printf "${BLUE}The RPLIDAR is a series of commercially available, low cost rotating laser scanners.${NC}\n"
    printf "${BLUE}This should work for the A1, A2, and A3 models.${NC}\n"
    printf "${BLUE}https://www.slamtec.com/en${NC}\n\n"
    if ! [[ ${TRAVIS} == "true" ]]; then # Never ask questions in Travis test environment
      read -n 1 -s -r -p "Press 'y' to install Slamtec RPLIDAR code " RESPONSE_TO_RPLIDAR_QUERY
    fi
    printf "\n"
  fi
  saveResponseData "${RESPONSE_TO_RPLIDAR_QUERY}" 'responseToRplidarQuery'

  RESPONSE_TO_KINECT_QUERY=$(jq -r '.responseToKinectQuery' "${SETUP_RESPONSE_FILE}")
  if [[ ${RESPONSE_TO_KINECT_QUERY} == 'null' ]]; then
    printf "\n${YELLOW}Do you want to install code for Xbox 360 Kinect?${NC}\n"
    printf "${BLUE}If you are using an Xbox 360 Kinect, extra code must be installed.${NC}\n"
    if ! [[ ${TRAVIS} == "true" ]]; then # Never ask questions in Travis test environment
      read -n 1 -s -r -p "Press 'y' to install Xbox 360 Kinect code " RESPONSE_TO_KINECT_QUERY
    fi
    printf "\n"
  fi
  saveResponseData "${RESPONSE_TO_KINECT_QUERY}" 'responseToKinectQuery'

  RESPONSE_TO_ASUS_XTION_QUERY=$(jq -r '.responseToAsusXtionQuery' "${SETUP_RESPONSE_FILE}")
  if [[ ${RESPONSE_TO_ASUS_XTION_QUERY} == 'null' ]]; then
    printf "\n${YELLOW}Do you want to install code for ASUS Xtion?${NC}\n"
    printf "${BLUE}If you are using an ASUS Xtion, extra code must be installed.${NC}\n"
    if ! [[ ${TRAVIS} == "true" ]]; then # Never ask questions in Travis test environment
      read -n 1 -s -r -p "Press 'y' to install ASUS Xtion code " RESPONSE_TO_ASUS_XTION_QUERY
    fi
    printf "\n"
  fi
  saveResponseData "${RESPONSE_TO_ASUS_XTION_QUERY}" 'responseToAsusXtionQuery'
fi

# End response collection section

printf "\n${YELLOW}[Installing additional Ubuntu and ROS Packages for Arlo]${NC}\n"
printf "${BLUE}This runs every time, in case new packages were added.${NC}\n"

# TODO: These packages do not exist in Jazzy:
#       E: Unable to locate package ros-jazzy-move-base

# TODO: Try removing some ROS packages that do not need to be installed independently,
#       because they are already dependencies of other things.

# TODO: Ensure these are also properly required by the Arlobot package,
# TODO: Test if rosdep installs them itself.
PACKAGE_TO_INSTALL_LIST=()
# ### Required Packages and Why ###
PACKAGE_TO_INSTALL_LIST+=(python3-argcomplete)
# python3-argcomplete - The Jazzy Install page recommends installing this.
PACKAGE_TO_INSTALL_LIST+=(git)
# git - allows for cloning of repositories
# TODO: move_base does not exist in ROS2
#       https://navigation.ros.org/about/ros1_comparison.html
#PACKAGE_TO_INSTALL_LIST+=("ros-${INSTALLING_ROS_DISTRO}-move-base")
# "ros-${INSTALLING_ROS_DISTRO}-move-base" - Required to build and use Arlobot ROS code.
PACKAGE_TO_INSTALL_LIST+=("ros-${INSTALLING_ROS_DISTRO}-twist-mux")
# twist-mux is used by the Arlobot cmd_vel_mux input controller.
#      It allows multiple cmd_vel topics to all coexist, and remaps the active on one
#           to the correct topic for output.
#      Without it, movement (twist) commands cannot be sent to the robot from ROS.
PACKAGE_TO_INSTALL_LIST+=("ros-${INSTALLING_ROS_DISTRO}-rosbridge-server")
# rosbridge is required for the Web interface to communicate with ROS
PACKAGE_TO_INSTALL_LIST+=("ros-${INSTALLING_ROS_DISTRO}-cv-bridge")
# cv-bridge is required by the costmap converter package
PACKAGE_TO_INSTALL_LIST+=("xvfb")
# xvfb is required for Cypress testing to work.

if ! [[ ${WORKSTATION_INSTALL} == "y" ]]; then
  # TODO: Some of these should probably be in package.xml, but that would require another round of testing.
  PACKAGE_TO_INSTALL_LIST+=(cron)
  # cron - required for running scheduled tasks
  PACKAGE_TO_INSTALL_LIST+=(moreutils)
  # moreutils - sponge is used by some of my scripts
  PACKAGE_TO_INSTALL_LIST+=(python3-pip)
  # python3-pip - Required to install Python tools for things such as
  #      USB Relay reader.
  if ! [[ ${TRAVIS} == "true" ]]; then # Upgrading openssh in Travis often fails due to timeouts.
    PACKAGE_TO_INSTALL_LIST+=(openssh-server)
  fi
  # openssh-server - required to SSH into robot remotely
  PACKAGE_TO_INSTALL_LIST+=(python3-serial)
  # python3-serial - required for ROS to talk to the Propeller board
  PACKAGE_TO_INSTALL_LIST+=(expect)
  # expect - required to get 'unbuffer' which is required by node to spawn ROS commands and get real time stdout data
  # http://stackoverflow.com/a/11337310
  # http://linux.die.net/man/1/unbuffer
  PACKAGE_TO_INSTALL_LIST+=(imagemagick)
  # imagemagick - used to grab screen shots of desktop for web local robot website display
  if [[ "${RESPONSE_TO_ASUS_XTION_QUERY}" == "y" ]] || [[ ${TRAVIS} == "true" ]]; then
    # Always test in Travis
    PACKAGE_TO_INSTALL_LIST+=("ros-${INSTALLING_ROS_DISTRO}-openni2-launch")
    # ros-noetic-openni2-launch - Required for ASUS Xtion to operate
  fi
  if [[ "$RESPONSE_TO_ASUS_XTION_QUERY" == "y" ]] || [[ "${RESPONSE_TO_KINECT_QUERY}" == "y" ]] || [[ ${TRAVIS} == "true" ]]; then
    # Always test in Travis
    PACKAGE_TO_INSTALL_LIST+=("ros-${INSTALLING_ROS_DISTRO}-depthimage-to-laserscan")
    # "ros-${INSTALLING_ROS_DISTRO}-depthimage-to-laserscan" - Required for ASUS Xtion and Kinect to operate
  fi
  if [[ "${RESPONSE_TO_KINECT_QUERY}" == "y" ]] || [[ ${TRAVIS} == "true" ]]; then
    # Always test in Travis
    # TODO: Uncomment if they ever release freenect_stack for Jazzy:
    #PACKAGE_TO_INSTALL_LIST+=("ros-${INSTALLING_ROS_DISTRO}-freenect-stack")
    echo "freenect_stack will be installed from source instead."
    # "ros-${INSTALLING_ROS_DISTRO}-freenect-stack" - Required for Kinect to operate
  fi
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

  # TODO: Confirm that Scanse Sweep needs this and add it to the "if scanse" section:
  # ros-${INSTALLING_ROS_DISTRO}-pointcloud-to-laserscan - used by Scanse Sweep

  # NOTE: If you are looking for a ROS package and wonder if it exists, but not for Noetic, check here:
  # http://repositories.ros.org/status_page/compare_melodic_noetic.html
fi

sudo apt install -y "${PACKAGE_TO_INSTALL_LIST[@]}"

if ! [[ -d ~/dev_ws/src ]]; then
  printf "\n${YELLOW}[Creating the ROS Development workspace and testing with colcon]${NC}\n"
  mkdir -p ~/dev_ws/src
  cd ~/dev_ws/
  colcon build
  # shellcheck source=/home/chrisl8/dev_ws/install/setup.bash
  source ~/dev_ws/install/setup.bash
fi

printf "\n${YELLOW}[Cloning or Updating git repositories]${NC}\n"
cd ~/dev_ws/src

printf "${BLUE}ArloBot repository${NC}\n"
if ! [[ -d ~/dev_ws/src/ArloBot ]]; then
  git clone -b jazzy https://github.com/chrisl8/ArloBot.git
else
  cd ~/dev_ws/src/ArloBot
  git fetch
  git checkout jazzy
  git pull
fi

if [[ "${RESPONSE_TO_XV11_QUERY}" == "y" ]] || [[ ${TRAVIS} == "true" ]]; then # Always test in Travis
  printf "\n${BLUE}Neato XV11 repository${NC}\n"
  # Only needed if you have an XV-11 "Neato" Scanner
  cd ~/dev_ws/src
  if ! [[ -d ~/dev_ws/src/xv_11_laser_driver ]]; then
    git clone -b noetic-devel https://github.com/chrisl8/xv_11_laser_driver.git
  else
    cd ~/dev_ws/src/xv_11_laser_driver
    git pull
  fi
fi

if [[ "${RESPONSE_TO_SWEEP_QUERY}" == "y" ]] || [[ ${TRAVIS} == "true" ]]; then
  # Always test in Travis
  printf "\n${BLUE}Scanse Sweep repository${NC}\n"
  # Only needed if you have a Scanse Sweep
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
  cd ~/dev_ws/src
  if ! [[ -d ~/dev_ws/src/sweep-ros ]]; then
    # Using my repository to fix compile issues.
    git clone https://github.com/chrisl8/sweep-ros.git
  else
    cd ~/dev_ws/src/sweep-ros
    git pull
  fi
fi

# TODO: Remove if they ever release freenect_stack for Noetic
if [[ "${RESPONSE_TO_KINECT_QUERY}" == "y" ]] || [[ ${TRAVIS} == "true" ]]; then
  # Always test in Travis
  printf "\n${BLUE}OpenKinect for Kinect${NC}\n"
  # If you have a Kinect. Noetic seems to be missing the package
  # https://github.com/ros-drivers/freenect_stack/issues/48#issuecomment-514020969
  if ! [[ -f /usr/local/lib/fakenect/libfakenect.so ]]; then
    cd
    git clone https://github.com/OpenKinect/libfreenect.git
    cd libfreenect
    mkdir build
    cd build
    cmake -L ..
    make
    sudo make install
  fi
  cd ~/dev_ws/src
  if ! [[ -d ~/dev_ws/src/freenect_stack ]]; then
    git clone https://github.com/ros-drivers/freenect_stack.git
  else
    cd ~/dev_ws/src/freenect_stack
    git pull
  fi
fi

if [[ "${RESPONSE_TO_RPLIDAR_QUERY}" == "y" ]] || [[ ${TRAVIS} == "true" ]]; then
  # Always test in Travis
  printf "\n${BLUE}Slamtec RPLIDAR${NC}\n"
  cd ~/dev_ws/src
  if ! [[ -d ~/dev_ws/src/rplidar_ros ]]; then
    git clone -b dev-ros2 https://github.com/Slamtec/rplidar_ros.git
  else
    cd ~/dev_ws/src/rplidar_ros
    git fetch
    git checkout dev-ros2
    git pull
  fi
fi

# TODO: Remove this if/when the apt package is released for jazzy.
# TODO: This won't build on Jazzy.
#printf "\n${BLUE}Costmap Converter Msgs${NC}\n"
#printf "${PURPLE}This is required by TEB Local Planner, but has not been released for ${INSTALLING_ROS_DISTRO}.${NC}\n"
#cd ~/dev_ws/src
#if ! [[ -d ~/dev_ws/src/costmap_converter ]]; then
#  git clone -b ros2 https://github.com/rst-tu-dortmund/costmap_converter.git
#else
#  cd ~/dev_ws/src/costmap_converter
#  git checkout ros2
#  git pull
#fi

# TODO: Remove this if/when the apt package is released for jazzy.
# TODO: This won't build without costmap converter, which I haven't made build on Jazzy.
#printf "\n${BLUE}TEB Local Planner${NC}\n"
#printf "${PURPLE}The TEB Local Planner has not been released for ${INSTALLING_ROS_DISTRO}.${NC}\n"
#cd ~/dev_ws/src
#if ! [[ -d ~/dev_ws/src/teb_local_planner ]]; then
#  git clone -b ros2-master https://github.com/rst-tu-dortmund/teb_local_planner.git
#else
#  cd ~/dev_ws/src/teb_local_planner
#  git checkout ros2-master
#  git pull
#fi

# TODO: Remove this if/when the apt package is released for jazzy.
# TODO: This won't build and doesn't matter either without TEB Local planner
#printf "\n${BLUE}TEB Local Planner Tutorials${NC}\n"
#printf "${PURPLE}The TEB Local Planner tutorials have not been ported to ${INSTALLING_ROS_DISTRO}.${NC}\n"
#printf "${PURPLE}The tutorials are not required, but handy for reference.${NC}\n"
#cd ~/dev_ws/src
#if ! [[ -d ~/dev_ws/src/teb_local_planner_tutorials ]]; then
#  git clone -b noetic-devel https://github.com/rst-tu-dortmund/teb_local_planner_tutorials.git
#else
#  cd ~/dev_ws/src/teb_local_planner_tutorials
#  git checkout noetic-devel
#  git pull
#fi

# TODO: Remove this if/when the apt package is released for jazzy.
printf "\n${BLUE}Slam Toolbox${NC}\n"
cd ~/dev_ws/src
if ! [[ -d ~/dev_ws/src/slam_toolbox ]]; then
  git clone https://github.com/SteveMacenski/slam_toolbox.git
else
  cd ~/dev_ws/src/slam_toolbox
  git pull
fi

# TODO: Remove this if/when the apt package is released for jazzy.
# Required by Nav2
printf "\n${BLUE}Minimal Turtlebot Simulation${NC}\n"
printf "${PURPLE}This is required by Navigation 2.${NC}\n"
cd ~/dev_ws/src
if ! [[ -d ~/dev_ws/src/nav2_minimal_turtlebot_simulation ]]; then
  git clone https://github.com/ros-navigation/nav2_minimal_turtlebot_simulation.git
else
  cd ~/dev_ws/src/nav2_minimal_turtlebot_simulation
  git pull
fi

# TODO: Remove this if/when the apt package is released for jazzy.
printf "\n${BLUE}Slam Toolbox${NC}\n"
cd ~/dev_ws/src
if ! [[ -d ~/dev_ws/src/navigation2 ]]; then
  git clone https://github.com/ros-navigation/navigation2.git
else
  cd ~/dev_ws/src/navigation2
  git pull
fi

export PIP_DISABLE_PIP_VERSION_CHECK=1

cd ~/dev_ws
printf "\n${YELLOW}[Installing dependencies for ROS build-from-source packages.]${NC}\n"
rosdep install -q -y -r --from-paths src --ignore-src --rosdistro ${INSTALLING_ROS_DISTRO}
printf "\n${YELLOW}[(Re)Building ROS Source files.]${NC}\n"
colcon build
# shellcheck source=/home/chrisl8/dev_ws/install/setup.bash
source ~/dev_ws/install/setup.bash

if [[ -f ~/.bashrc ]]; then
  if [[ ${WORKSTATION_INSTALL} == "y" ]]; then
    if ! (grep ROS_MASTER_URI ~/.bashrc >/dev/null); then
      if ! [[ ${ROBOT_IP_ADDRESS} ]]; then
        read -rp "What is the host name or IP of your robot? " ROBOT_IP_ADDRESS
      fi
      sh -c "echo \"export ROS_MASTER_URI=http://${ROBOT_IP_ADDRESS}:11311\" >> ~/.bashrc"
    fi
  fi
  if ! (grep ROS_HOSTNAME ~/.bashrc >/dev/null); then
    printf "\n${YELLOW}[Setting the ROS_HOSTNAME in your .bashrc file]${NC}\n"
    sh -c "echo \"export ROS_HOSTNAME=$(uname -n).local\" >> ~/.bashrc"
  fi
  if ! (grep ROSLAUNCH_SSH_UNKNOWN ~/.bashrc >/dev/null); then
    printf "\n${YELLOW}[Setting the ROSLAUNCH_SSH_UNKNOWN in your .bashrc file]${NC}\n"
    sh -c "echo \"export ROSLAUNCH_SSH_UNKNOWN=1\" >> ~/.bashrc"
  fi
  if ! (grep "dev_ws/install/setup.bash" ~/.bashrc >/dev/null); then
    printf "\n${YELLOW}[Setting the ROS setup.bash source call in your .bashrc file]${NC}\n"
    sh -c "echo \"source ~/dev_ws/install/setup.bash\" >> ~/.bashrc"
  fi
  if ! (grep "${HOME}/dev_ws/src/ArloBot/scripts" ~/.bashrc >/dev/null); then
    printf "\n${YELLOW}[Adding ArloBot Scripts folder to your path in .bashrc]${NC}\n"
    sh -c "echo \"export PATH=\\\$PATH:${HOME}/dev_ws/src/ArloBot/scripts\" >> ~/.bashrc"
  fi
fi

if [[ -f ~/.zshrc ]]; then
  if [[ ${WORKSTATION_INSTALL} == "y" ]]; then
    if ! (grep ROS_MASTER_URI ~/.bashrc >/dev/null); then
      if ! [[ ${ROBOT_IP_ADDRESS} ]]; then
        read -rp "What is the host name or IP of your robot? " ROBOT_IP_ADDRESS
      fi
      sh -c "echo \"export ROS_MASTER_URI=http://${ROBOT_IP_ADDRESS}:11311\" >> ~/.zshrc"
    fi
  fi
  if ! (grep ROS_HOSTNAME ~/.zshrc >/dev/null); then
    printf "\n${YELLOW}[Setting the ROS_HOSTNAME in your .zshrc file]${NC}\n"
    sh -c "echo \"export ROS_HOSTNAME=$(uname -n).local\" >> ~/.zshrc"
  fi
  if ! (grep ROSLAUNCH_SSH_UNKNOWN ~/.zshrc >/dev/null); then
    printf "\n${YELLOW}[Setting the ROSLAUNCH_SSH_UNKNOWN in your .zshrc file]${NC}\n"
    sh -c "echo \"export ROSLAUNCH_SSH_UNKNOWN=1\" >> ~/.zshrc"
  fi
  if ! (grep "dev_ws/install/setup.zsh" ~/.zshrc >/dev/null); then
    printf "\n${YELLOW}[Setting the ROS setup.zsh source call in your .zshrc file]${NC}\n"
    sh -c "echo \"source ~/dev_ws/install/setup.zsh\" >> ~/.zshrc"
  fi
  if ! (grep "${HOME}/dev_ws/src/ArloBot/scripts" ~/.zshrc >/dev/null); then
    printf "\n${YELLOW}[Adding ArloBot Scripts folder to your path in .zshrc]${NC}\n"
    sh -c "echo \"export PATH=\\\$PATH:${HOME}/dev_ws/src/ArloBot/scripts\" >> ~/.zshrc"
  fi
fi

if ! [[ ${WORKSTATION_INSTALL} == "y" ]]; then
  # For 8-CH USB Relay board:
  pip3 install pylibftdi --break-system-packages
  # Required by pylibftdi
  # https://pylibftdi.readthedocs.io/en/0.15.0/installation.html
  if ! [[ -f /etc/udev/rules.d/99-libftdi.rules ]]; then
    printf "\n${YELLOW}[Adding required sudo rule for pylibftdi to access USB based serial ports.]${NC}\n"
    sudo "${HOME}/dev_ws/src/ArloBot/scripts/addRuleForUSBRelayBoard.sh"
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

  wget -qO- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.3/install.sh | bash
  export NVM_DIR="${HOME}/.nvm"
  # shellcheck source=/home/chrisl8/.nvm/nvm.sh
  [[ -s "$NVM_DIR/nvm.sh" ]] && . "$NVM_DIR/nvm.sh" # This loads nvm

  export NVM_SYMLINK_CURRENT=true
  if ! (grep NVM_SYMLINK_CURRENT ~/.bashrc >/dev/null); then
    printf "\n${YELLOW}[Setting the NVM current environment in your .bashrc file]${NC}\n"
    sh -c "echo \"export NVM_SYMLINK_CURRENT=true\" >> ~/.bashrc"
  fi
  nvm install --lts
  nvm alias default "lts/*"

  printf "\n${YELLOW}[Grabbing/Updating global dependencies for node packages]${NC}\n"
  printf "${BLUE}You may get some errors here, that is normal. As long as things work, it is OK.$NC\n"
  cd
  printf "\n${YELLOW}[PM2 for running Robot service]$NC\n"
  npm install -g pm2
  printf "\n${YELLOW}[Log Streamer for Website]$NC\n"
  npm install -g log.io
  printf "\n${YELLOW}[Log.io File Watcher for Log.io Log Streamer]$NC\n"
  npm install -g log.io-file-input

  if [[ -d ~/dev_ws/src/ArloBot/Log.io ]]; then
    printf "\n"
    printf "${BLUE}Removing old Log.io Install${NC}\n"
    rm -rf "${HOME}/dev_ws/src/ArloBot/Log.io"
  fi
fi

cd "${HOME}/dev_ws/src/ArloBot/node"
printf "\n${YELLOW}[Grabbing node dependencies for scripts]${NC}\n"
printf "${BLUE}You may get some errors here, that is normal. As long as things work, it is OK.$NC\n"
npm ci

if ! [[ ${WORKSTATION_INSTALL} == "y" ]]; then
  cd "${HOME}/dev_ws/src/ArloBot/website"
  printf "\n${YELLOW}[Grabbing node dependencies for React website]${NC}\n"
  npm ci
  printf "\n${YELLOW}[Building React website]${NC}\n"
  npm run build

  cd "${HOME}/dev_ws/src/ArloBot/cypress-tests"
  printf "\n${YELLOW}[Installing Cypress.io for Tests]$NC\n"
  npm ci

  if ! (command -v mjpg_streamer >/dev/null); then
    printf "\n${YELLOW}[Installing mjpg_streamer for Web Page camera viewing]${NC}\n"
    cd "${HOME}/dev_ws/src/ArloBot/"
    if ! [[ -d ${HOME}/dev_ws/src/ArloBot/mjpg-streamer ]]; then
      git clone https://github.com/jacksonliam/mjpg-streamer.git
    else
      cd "${HOME}/dev_ws/src/ArloBot/mjpg-streamer"
      git pull
    fi
    cd "${HOME}/dev_ws/src/ArloBot/mjpg-streamer/mjpg-streamer-experimental"
    make distclean
    make
    sudo make install
    # Removing install files, as they are not needed anymore.
    cd "${HOME}/dev_ws/src/ArloBot/"
    rm -rf mjpg-streamer
    # See scripts/streamVideoTest.sh for details on mjpg_streamer usage.
  fi
fi

if [[ -e ${ARLO_HOME}/personalDataForBehavior.json ]]; then
  node "${HOME}/dev_ws/src/ArloBot/node/personalData.js"
else
  printf "\n"
  cp "${HOME}/dev_ws/src/ArloBot/scripts/dotarlobot/personalDataForBehavior.json" "${ARLO_HOME}/"
  printf "${GREEN}A brand new ${RED}${ARLO_HOME}/personalDataForBehavior.json${GREEN} file has been created,${NC}\n"
  printf "${LIGHT_PURPLE}Please edit this file to customize according to your robot!${NC}\n"
fi

if ! [[ ${WORKSTATION_INSTALL} == "y" ]]; then
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
    printf "${RED}You may have to reboot before you can use the Propeller Board.${NC}\n"
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
fi

if ! [[ -e /usr/share/PropWare/include/arlodrive.h ]]; then
  printf "\n${YELLOW}[Setting up PropWare and PropGCC for putting code on Activity Board.]${NC}\n"
  printf "${BLUE}Parallax no longer supports Linux so we are using some third party tools.${NC}\n"
  printf "${BLUE}https://david.zemon.name/PropWare${NC}\n"
  cd /tmp
  # NOTE: This is the original location, but it has gone dark
  # wget -O propware_3.0.0.224-1_all.deb https://ci.zemon.name/repository/download/PropWare_Develop/3817:id/propware_3.0.0.224-1_all.deb?guest=1
  wget -O propware_3.0.0.224-1_all.deb https://www.dropbox.com/s/12l51adhwge1y43/propware_3.0.0.224-1_all.deb?dl=1
  sudo dpkg -i /tmp/propware_3.0.0.224-1_all.deb
  rm /tmp/propware_3.0.0.224-1_all.deb
fi

if ! [[ -d /opt/parallax ]]; then
  printf "\n${YELLOW}[Installing PropGCC, which is required by PropWare.]${NC}\n"
  cd /tmp
  # NOTE: This is the original location, but it has gonedark
  # wget -O propellergcc-alpha_v1_9_0-gcc4-linux-x64.tar.gz https://ci.zemon.name/repository/download/PropGCC5_Gcc4linuxX64/3620:id/propellergcc-alpha_v1_9_0-gcc4-linux-x64.tar.gz?guest=1
  # NOTE: I also have this stored in my Dropbox. If the above link dies use:
  wget -O propellergcc-alpha_v1_9_0-gcc4-linux-x64.tar.gz https://www.dropbox.com/s/sccr5cs46hgrlwd/propellergcc-alpha_v1_9_0-gcc4-linux-x64.tar.gz?dl=1
  sudo cp /tmp/propellergcc-alpha_v1_9_0-gcc4-linux-x64.tar.gz /opt
  rm /tmp/propellergcc-alpha_v1_9_0-gcc4-linux-x64.tar.gz
  cd /opt
  sudo tar xf propellergcc-alpha_v1_9_0-gcc4-linux-x64.tar.gz
  sudo rm /opt/propellergcc-alpha_v1_9_0-gcc4-linux-x64.tar.gz
  cd # Get out of /opt just for safety
fi

if ! [[ -e ${ARLO_HOME}/per_robot_settings_for_propeller_c_code.h ]]; then
  cp "${HOME}/dev_ws/src/ArloBot/PropellerCodeForArloBot/dotfiles/per_robot_settings_for_propeller_c_code.h" "${ARLO_HOME}"
fi

if ! [[ -e ${ARLO_HOME}/per_robot_settings_for_propeller_2nd_board.h ]]; then
  cp "${HOME}/dev_ws/src/ArloBot/PropellerCodeForArloBot/dotfiles/per_robot_settings_for_propeller_2nd_board.h" "${ARLO_HOME}"
fi

printf "\n${YELLOW}[Test Compiling Propeller Code.]${NC}\n"
printf "${BLUE}You will need to load this code onto your Propeller board after the setup is done.${NC}\n"

function testMake() {
  printf " ${BLUE}- ${1}${NC}\n"
  cd "${HOME}/dev_ws/src/ArloBot/PropellerCodeForArloBot/${1}"
  rm -rf bin
  if ! [[ -d bin ]]; then
    mkdir bin
  fi
  cd bin
  cmake -DCMAKE_MODULE_PATH=/usr/share/PropWare/CMakeModules -G "Unix Makefiles" ..
  make

}
testMake ROSInterfaceForArloBot
testMake MotorResponseTesting
testMake 2ndBoardCode
testMake Calib
cd

if ! [[ ${WORKSTATION_INSTALL} == "y" ]]; then
  if [[ -e ${ARLO_HOME}/arlobot.yaml ]]; then
    if ! (diff "${HOME}/dev_ws/src/ArloBot/arlobot_ros/param/arlobot.yaml" "${ARLO_HOME}/arlobot.yaml" >/dev/null); then
      printf "\n${GREEN}The ${RED}arlobot.yaml${GREEN} file in the repository is different from the one${NC}\n"
      printf "${GREEN}in your local settings.${NC}\n"
      printf "${GREEN}This is expected, but just in case, please look over the differences,${NC}\n"
      printf "${GREEN}and see if you need to copy in any new settings, or overwrite the file completely:${NC}\n"
      diff "${HOME}/dev_ws/src/ArloBot/arlobot_ros/param/arlobot.yaml" "${ARLO_HOME}/arlobot.yaml" || true
      cp -i "${HOME}/dev_ws/src/ArloBot/arlobot_ros/param/arlobot.yaml" "${ARLO_HOME}/"
      printf "\n"
    fi
  else
    printf "\n"
    cp "${HOME}/dev_ws/src/ArloBot/arlobot_ros/param/arlobot.yaml" "${ARLO_HOME}/"
    printf "${GREEN}A brand new ${RED}${ARLO_HOME}/arlobot.yaml${GREEN} file has been created,${NC}\n"
    printf "${LIGHT_PURPLE}Please edit this file to customize according to your robot!${NC}\n"
  fi

  for i in "${HOME}/dev_ws/src/ArloBot/PropellerCodeForArloBot/dotfiles/"*; do
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

  if ! (crontab -l >/dev/null 2>&1) || ! (crontab -l | grep startpm2 >/dev/null 2>&1); then
    printf "\n${YELLOW}[Adding cron job to start web server on system reboot.]${NC}\n"
    # https://stackoverflow.com/questions/4880290/how-do-i-create-a-crontab-through-a-script
    (
      echo "@reboot ${HOME}/dev_ws/src/ArloBot/startpm2.sh > ${HOME}/crontab.log"
    ) | crontab -
  fi

  printf "\n${LIGHTPURPLE}[Flushing PM2 logs and starting/restarting web server.]${NC}\n"
  pm2 update
  pm2 flush
  if ! pm2 restart Robot; then
    "${HOME}/dev_ws/src/ArloBot/startpm2.sh"
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

    # Special notices for the developer himself to keep his stuff up to date. =)
    cd "${HOME}/dev_ws/src/ArloBot/node"
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
    cd "${HOME}/dev_ws/src/ArloBot/website"
    npm outdated || true # Informational, do not crash  script
    printf "${YELLOW}in cypress tests/:${NC}\n"
    cd "${HOME}/dev_ws/src/ArloBot/cypress-tests"
    npm outdated || true # Informational, do not crash  script
    printf "${PURPLE}-------------------------------------------------------${NC}\n"

    printf "${YELLOW}Diffing your param files to defaults where available:${NC}\n"
    printf "${BLUE}teb_local_planner_params:${NC}\n"
    diff "${HOME}/dev_ws/src/teb_local_planner_tutorials/cfg/diff_drive/teb_local_planner_params.yaml" "${HOME}/dev_ws/src/ArloBot/arlobot_ros/param/teb_local_planner_params.yaml" || true
    printf "${BLUE}twist_mux_locks:${NC}\n"
    diff /opt/ros/${INSTALLING_ROS_DISTRO}/share/twist_mux/config/twist_mux_locks.yaml "${HOME}/dev_ws/src/ArloBot/arlobot_ros/param/twist_mux_locks.yaml" || true
    printf "${BLUE}twist_mux_topics:${NC}\n"
    diff /opt/ros/${INSTALLING_ROS_DISTRO}/share/twist_mux/config/twist_mux_topics.yaml "${HOME}/dev_ws/src/ArloBot/arlobot_ros/param/twist_mux_topics.yaml" || true
    printf "${BLUE}mapper_params_localization:${NC}\n"
    diff "${HOME}/dev_ws/src/slam_toolbox/slam_toolbox/config/mapper_params_localization.yaml" "${HOME}/dev_ws/src/ArloBot/arlobot_ros/param/mapper_params_localization.yaml" || true
    printf "${BLUE}mapper_params_online_async:${NC}\n"
    diff "${HOME}/dev_ws/src/slam_toolbox/slam_toolbox/config/mapper_params_online_async.yaml" "${HOME}/dev_ws/src/ArloBot/arlobot_ros/param/mapper_params_online_async.yaml" || true
  fi

  printf "\n${PURPLE}Anytime you want to update ArloBot code from the web you can run this same script again. It will pull down and compile new code without wiping out custom configs in ~/.arlobot. I run this script myself almost every day.${NC}\n"

  printf "\n${YELLOW}-----------------------------------${NC}\n"
  printf "${YELLOW}ALL DONE! REBOOT, EDIT FILES, AND START TESTING!${NC}\n\n"
  printf "${GREEN}Remember to edit the config files in ~/.arlobot${NC}\n\n"
  printf "${LIGHTCYAN}Go to ${LIGHTBLUE}http://$(node "${HOME}/dev_ws/src/ArloBot/node/ipAddress.js"):$(jq '.webServerPort' "${ARLO_HOME}/personalDataForBehavior.json")${LIGHTCYAN} to see the Arlobot web interface.${NC}\n"
  printf "\n"
  printf "${GREEN}Look at README.md for testing ideas.${NC}\n"

  printf "\n${YELLOW}------------------------------------------------------------${NC}\n"
  printf "${YELLOW}Remember: You MUST install the Propeller code on your Propeller board too!${NC}\n"
  printf "${BLUE}You can run install_Propeller_code.sh to perform the install,${NC}\n"
  printf "${BLUE}and PropellerSerialTest.sh to test it.${NC}\n"
  printf "${GREEN}See: ${BLUE}https://ekpyroticfrood.net/?p=551${NC}\n"
  printf "${GREEN}for more information on installing code on your Propeller board.${NC}\n"
  printf "${YELLOW}------------------------------------------------------------${NC}\n"
fi

INSTALL_FINISHED="true"
