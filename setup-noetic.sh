#!/usr/bin/env bash
# shellcheck disable=SC2059 disable=SC2129
# ROS Arlobot Automated Install

INSTALLING_ROS_DISTRO=noetic

# Run this straight off of github like this:
# bash <(wget -qO- --no-cache https://raw.githubusercontent.com/chrisl8/ArloBot/noetic/setup-noetic.sh)

#   TESTING
#
# Testing install with Docker:
#
# You can Test this with Docker by installing Docker, then pulling down the Ubuntu 20.04 (Focal) image:
# docker pull ubuntu:focal
# cd ~/catkin_ws/src/ArloBot
#
# Then either kick it off all in one shot:
# docker run -ti -v $PWD:/home/user ubuntu:focal /bin/bash -c "export CI=true;/home/user/setup-noetic.sh"
#
# Or start an interactive shell in Docker and run it, with the ability to make changes and start it again when it finishes:
# docker run -ti -v $PWD:/home/user ubuntu:focal /bin/bash
# export CI=true
# /home/user/setup-noetic.sh
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

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
LIGHTCYAN='\033[1;36m'
LIGHTBLUE='\033[1;34m'
LIGHT_PURPLE='\033[1;35m'
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
    printf "${YELLOW}If this persists, please copy the output of the script above showing what the error was and file an issue against the repository at https://github.com/chrisl8/ArloBot/issues${NC}\n"
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
  # When the prerequisite install installs the tzdat package,
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

  # Installing this now appears to prevent it from hanging up the Docker setup later.
  apt install -y keyboard-configuration

  DOCKER_TEST_INSTALL=true
fi

version=$(lsb_release -sc)
architecture=$(uname -m)

printf "\n${YELLOW}[Checking the Ubuntu version]${NC}\n"
printf "${LIGHTBLUE}Ubuntu ${version} found${NC}\n"
case ${version} in
"focal") ;;

*)
  printf "${RED}[This script will only work on Ubuntu Focal (20.04)]${NC}\n"
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
  printf "\n"
fi

printf "${YELLOW}[Adding/Updating ROS Repository Key]${NC}\n"
# As explained in the official ROS install instruction
#      http://wiki.ros.org/noetic/Installation/Ubuntu
# The pool options are listed here: https://sks-keyservers.net/overview-of-pools.php
# https://github.com/tianon/gosu/issues/39#issuecomment-362544059
for server in ha.pool.sks-keyservers.net \
  hkp://p80.pool.sks-keyservers.net:80 \
  keyserver.ubuntu.com \
  hkp://keyserver.ubuntu.com:80 \
  pgp.mit.edu; do
  sudo apt-key adv --keyserver "$server" --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && break || echo "Trying new server..."
  #gpg --keyserver "$server" --recv-keys B42F6819007F00F88E364FD4036A9C25BF357DD4 && break || echo "Trying new server..."
done

if ! [[ -e /etc/apt/sources.list.d/ros-latest.list ]]; then
  printf "\n${YELLOW}[Adding the ROS repository]${NC}\n"
  # As explained in the official ROS install instruction
  #      http://wiki.ros.org/noetic/Installation/Ubuntu
  sudo sh -c "echo \"deb http://packages.ros.org/ros/ubuntu ${version} main\" > /etc/apt/sources.list.d/ros-latest.list"
fi

printf "\n${YELLOW}[Updating & upgrading all existing Ubuntu packages]${NC}\n"
sudo apt update
sudo apt upgrade -y

# Determine whether to install ros desktop-full or base, based on the presence of X windows components.
ROS_META_PACKAGE=ros-base
if (dpkg -l | grep xserver-xorg-core); then
  ROS_META_PACKAGE=desktop-full
fi

# This should follow the official ROS install instructions closely.
#      http://wiki.ros.org/noetic/Installation/Ubuntu
# That is why there is a separate section for extra packages that I need for Arlo.
# Note, while we are NOT building ROS from source, we are building SOME THINGS from source,
# and hence must also follow the the instructions to set up a build environment
#      http://wiki.ros.org/noetic/Installation/Source
if ! (dpkg -s ros-${INSTALLING_ROS_DISTRO}-${ROS_META_PACKAGE} | grep "Status: install ok installed" &>/dev/null) || ! (command -v rosdep) || ! (command -v rosinstall_generator); then
  printf "\n${YELLOW}[Installing ROS]${NC}\n"
  sudo apt install -y ros-${INSTALLING_ROS_DISTRO}-${ROS_META_PACKAGE}
  printf "${YELLOW}[ROS installed!]${NC}\n"
  printf "\n${LIGHTBLUE}Installing ROS Dependencies for building packages${NC}\n"
  sudo apt install -y python3-rosdep python3-rosinstall-generator python3-vcstool build-essential
  if ! [[ -e /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    printf "\n${LIGHTBLUE}Running rosdep init . . . ${NC}\n"
    sudo sh -c "rosdep init"
  fi
  printf "${LIGHTBLUE}Running rosdep update . . .${NC}\n"
  rosdep update
  # shellcheck source=/opt/ros/noetic/setup.bash
  source /opt/ros/${INSTALLING_ROS_DISTRO}/setup.bash
  # END Official ROS Install section
fi

# In case .bashrc wasn't set up, or you didn't reboot
if ! (command -v catkin_make >/dev/null); then
  # shellcheck source=/opt/ros/noetic/setup.bash
  source /opt/ros/${INSTALLING_ROS_DISTRO}/setup.bash
fi

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
  printf "${LIGHTBLUE}Your answers depend on what hardware you have.${NC}\n"
  printf "${LIGHTBLUE}It will not hurt to install things you do not need.${NC}\n\n"
  printf "${LIGHTBLUE}Your responses will be saved and used on future runs of this script.${NC}\n\n"
  printf "${LIGHTBLUE}If you want to erase your answers and answer again run:${NC}\n"
  printf "${LIGHTBLUE}rm ${SETUP_RESPONSE_FILE}${NC}\n"
  printf "${LIGHTBLUE}before running this script.${NC}\n\n"
  if ! [[ ${CI} == "true" ]]; then # Never ask questions in Travis test environment
    read -n 1 -s -r -p "Press any key to continue"
  fi
fi

# Ask questions
WORKSTATION_INSTALL=$(jq -r '.responseToWorkstationQuery' "${SETUP_RESPONSE_FILE}")
if [[ ${WORKSTATION_INSTALL} == 'null' ]]; then
  printf "\n${YELLOW}Do you want this to be a WORKSTATION ONLY install?${NC}\n"
  printf "${LIGHTBLUE}A 'workstation' is a system that is NOT your robot,${NC}\n"
  printf "${LIGHTBLUE}but that you want to develop on and/or run ROS tools remotely${NC}\n\n"
  printf "${LIGHTBLUE}to operate and debug your robot.${NC}\n\n"
  printf "${LIGHTBLUE}This includes things like running RVIZ to view/control navigation and map making.${NC}\n\n"
  if ! [[ ${CI} == "true" ]]; then # Never ask questions in Travis test environment
    read -n 1 -s -r -p "Press 'y' to set as Workstation ONLY install " WORKSTATION_INSTALL
  fi
  printf "\n"
fi
saveResponseData "${WORKSTATION_INSTALL}" 'responseToWorkstationQuery'

if ! [[ ${WORKSTATION_INSTALL} == "y" ]]; then
  RESPONSE_TO_MYCROFT_QUERY=$(jq -r '.responseToMycroftQuery' "${SETUP_RESPONSE_FILE}")
  if [[ ${RESPONSE_TO_MYCROFT_QUERY} == 'null' ]]; then
    printf "\n${YELLOW}Do you want to install Mycroft on the Robot?${NC}\n"
    printf "${LIGHTBLUE}Mycroft can be used to talk to your robot, and have it talk to you.${NC}\n"
    printf "${LIGHTBLUE}Mycroft can be heavy on system resources on older systems though.${NC}\n\n"
    printf "${LIGHTBLUE}Mycroft is NOT hardware related, it is a software application.${NC}\n\n"
    if ! [[ ${CI} == "true" ]]; then # Never ask questions in Travis test environment
      read -n 1 -s -r -p "Press 'y' to install Mycroft " RESPONSE_TO_MYCROFT_QUERY
    fi
    printf "\n"
  fi
  saveResponseData "${RESPONSE_TO_MYCROFT_QUERY}" 'responseToMycroftQuery'

  RESPONSE_TO_SWEEP_QUERY=$(jq -r '.responseToSweepQuery' "${SETUP_RESPONSE_FILE}")
  if [[ ${RESPONSE_TO_SWEEP_QUERY} == 'null' ]]; then
    printf "\n${YELLOW}Do you want to install code for Scanse Sweep?${NC}\n"
    printf "${LIGHTBLUE}The Scanse Sweep is a rotating laser scanner.${NC}\n"
    printf "${LIGHTBLUE}It is no longer available.${NC}\n\n"
    printf "${LIGHTBLUE}https://scanse.io/home/${NC}\n\n"
    if ! [[ ${CI} == "true" ]]; then # Never ask questions in Travis test environment
      read -n 1 -s -r -p "Press 'y' to install Scanse Sweep code " RESPONSE_TO_SWEEP_QUERY
    fi
    printf "\n"
  fi
  saveResponseData "${RESPONSE_TO_SWEEP_QUERY}" 'responseToSweepQuery'

  RESPONSE_TO_XV11_QUERY=$(jq -r '.responseToXv11Query' "${SETUP_RESPONSE_FILE}")
  if [[ ${RESPONSE_TO_XV11_QUERY} == 'null' ]]; then
    printf "\n${YELLOW}Do you want to install code for Neato XV11?${NC}\n"
    printf "${LIGHTBLUE}The XV11 was a rotating laser scanner pulled from old vacuum cleaners.${NC}\n"
    printf "${LIGHTBLUE}If you have one you will need this code.${NC}\n\n"
    if ! [[ ${CI} == "true" ]]; then # Never ask questions in Travis test environment
      read -n 1 -s -r -p "Press 'y' to install Neato XV11 code " RESPONSE_TO_XV11_QUERY
    fi
    printf "\n"
  fi
  saveResponseData "${RESPONSE_TO_XV11_QUERY}" 'responseToXv11Query'

  RESPONSE_TO_RPLIDAR_QUERY=$(jq -r '.responseToRplidarQuery' "${SETUP_RESPONSE_FILE}")
  if [[ ${RESPONSE_TO_RPLIDAR_QUERY} == 'null' ]]; then
    printf "\n${YELLOW}Do you want to install code for Slamtec RPLIDAR?${NC}\n"
    printf "${LIGHTBLUE}The RPLIDAR is a series of commercially available, low cost rotating laser scanners.${NC}\n"
    printf "${LIGHTBLUE}This should work for the A1, A2, and A3 models.${NC}\n"
    printf "${LIGHTBLUE}https://www.slamtec.com/en${NC}\n\n"
    if ! [[ ${CI} == "true" ]]; then # Never ask questions in Travis test environment
      read -n 1 -s -r -p "Press 'y' to install Slamtec RPLIDAR code " RESPONSE_TO_RPLIDAR_QUERY
    fi
    printf "\n"
  fi
  saveResponseData "${RESPONSE_TO_RPLIDAR_QUERY}" 'responseToRplidarQuery'

  RESPONSE_TO_KINECT_QUERY=$(jq -r '.responseToKinectQuery' "${SETUP_RESPONSE_FILE}")
  if [[ ${RESPONSE_TO_KINECT_QUERY} == 'null' ]]; then
    printf "\n${YELLOW}Do you want to install code for Xbox 360 Kinect?${NC}\n"
    printf "${LIGHTBLUE}If you are using an Xbox 360 Kinect, extra code must be installed.${NC}\n"
    if ! [[ ${CI} == "true" ]]; then # Never ask questions in Travis test environment
      read -n 1 -s -r -p "Press 'y' to install Xbox 360 Kinect code " RESPONSE_TO_KINECT_QUERY
    fi
    printf "\n"
  fi
  saveResponseData "${RESPONSE_TO_KINECT_QUERY}" 'responseToKinectQuery'

  RESPONSE_TO_ASUS_XTION_QUERY=$(jq -r '.responseToAsusXtionQuery' "${SETUP_RESPONSE_FILE}")
  if [[ ${RESPONSE_TO_ASUS_XTION_QUERY} == 'null' ]]; then
    printf "\n${YELLOW}Do you want to install code for ASUS Xtion?${NC}\n"
    printf "${LIGHTBLUE}If you are using an ASUS Xtion, extra code must be installed.${NC}\n"
    if ! [[ ${CI} == "true" ]]; then # Never ask questions in Travis test environment
      read -n 1 -s -r -p "Press 'y' to install ASUS Xtion code " RESPONSE_TO_ASUS_XTION_QUERY
    fi
    printf "\n"
  fi
  saveResponseData "${RESPONSE_TO_ASUS_XTION_QUERY}" 'responseToAsusXtionQuery'
fi

# End response collection section

printf "\n${YELLOW}[Installing additional Ubuntu and ROS Packages for Arlo]${NC}\n"
printf "${LIGHTBLUE}This runs every time, in case new packages were added.${NC}\n"

# NOTE: In theory these are/will also installed when rosdep install is run,
#       but ensuring they are installed here is fine too.
PACKAGE_TO_INSTALL_LIST=()
# ### Required Packages and Why ###
PACKAGE_TO_INSTALL_LIST+=(git)
# git - allows for cloning of repositories
if [[ "${ROS_META_PACKAGE}" == "desktop-full" ]]; then
  PACKAGE_TO_INSTALL_LIST+=("xvfb")
  # xvfb is required for Cypress testing to work.
fi
if ! [[ ${WORKSTATION_INSTALL} == "y" ]]; then
  PACKAGE_TO_INSTALL_LIST+=(wget)
  # wget - This is almost certainly already installed, but if not, this setup will fail.
  PACKAGE_TO_INSTALL_LIST+=(moreutils)
  # moreutils - sponge is used by some of my scripts
  PACKAGE_TO_INSTALL_LIST+=(python3-pip)
  # python3-pip - Required to install Python tools for things such as
  #      USB Relay reader.
  if ! [[ ${CI} == "true" ]]; then # Upgrading openssh in Travis often fails due to timeouts.
    PACKAGE_TO_INSTALL_LIST+=(openssh-server)
  fi
  # openssh-server - required to SSH into robot remotely
  PACKAGE_TO_INSTALL_LIST+=(python3-serial)
  # python3-serial - required for ROS to talk to the Propeller board
  PACKAGE_TO_INSTALL_LIST+=(expect)
  # expect - required to get 'unbuffer' which is required by node to spawn ROS commands and get real time stdout data
  # http://stackoverflow.com/a/11337310
  # http://linux.die.net/man/1/unbuffer
  if [[ "${ROS_META_PACKAGE}" == "desktop-full" ]]; then
    PACKAGE_TO_INSTALL_LIST+=(imagemagick)
    # imagemagick - used to grab screen shots of desktop for web local robot website display
  fi
  if [[ "${RESPONSE_TO_ASUS_XTION_QUERY}" == "y" ]] || [[ ${CI} == "true" ]]; then
    # Always test in Travis
    PACKAGE_TO_INSTALL_LIST+=("ros-${INSTALLING_ROS_DISTRO}-openni2-launch")
    # ros-noetic-openni2-launch - Required for ASUS Xtion to operate
  fi
  if [[ "$RESPONSE_TO_ASUS_XTION_QUERY" == "y" ]] || [[ "${RESPONSE_TO_KINECT_QUERY}" == "y" ]] || [[ ${CI} == "true" ]]; then
    # Always test in Travis
    PACKAGE_TO_INSTALL_LIST+=("ros-${INSTALLING_ROS_DISTRO}-depthimage-to-laserscan")
    # "ros-${INSTALLING_ROS_DISTRO}-depthimage-to-laserscan" - Required for ASUS Xtion and Kinect to operate
  fi
  if [[ "${RESPONSE_TO_KINECT_QUERY}" == "y" ]] || [[ ${CI} == "true" ]]; then
    # Always test in Travis
    # TODO: Uncomment if they ever release freenect_stack for Noetic
    #PACKAGE_TO_INSTALL_LIST+=("ros-${INSTALLING_ROS_DISTRO}-freenect-stack")
    echo "freenect_stack will be installed from source instead."
    # "ros-${INSTALLING_ROS_DISTRO}-freenect-stack" - Required for Kinect to operate
  fi
  PACKAGE_TO_INSTALL_LIST+=(fswebcam)
  # fswebcam - Used for streaming a camera to the website. Camera streaming will not work without it.
  PACKAGE_TO_INSTALL_LIST+=(zbar-tools)
  # zbar-tools - Used by node service to read QR codes via on board camera
  if [[ ${architecture} == "x86_64" ]]; then
    PACKAGE_TO_INSTALL_LIST+=(libftdi1-dev)
  else
    PACKAGE_TO_INSTALL_LIST+=(libftdi-dev)
  fi
  # libftdi1-dev - required by pylibftdi for talking to USB based serial boards like relay boards, etc.
  #           https://pylibftdi.readthedocs.io/en/0.18.0/installation.html
  #      For 8-CH USB Relay board:
  #           Reference: https://code.google.com/p/drcontrol/wiki/Install_RaspberryPi">https://code.google.com/p/drcontrol/wiki/Install_RaspberryPi
  #           TEST:
  #           python -m pylibftdi.examples.list_devices
  #           Should return:
  #           FTDI:FT245R USB FIFO:A9026EI5
  #           If you have a USB Relay board attached via USB.
  # I'm not entirely sure if libftdi1-dev or libftdi-dev is correct for x86, but I DO KNOW that libftdi1-dev segfaults on Pi.

  # TODO: Confirm that Scanse Sweep needs this and add it to the "if scanse" section:
  # ros-${INSTALLING_ROS_DISTRO}-pointcloud-to-laserscan - used by Scanse Sweep

  # NOTE: If you are looking for a ROS package and wonder if it exists, but not for Noetic, check here:
  # http://repositories.ros.org/status_page/compare_melodic_noetic.html
  PACKAGE_TO_INSTALL_LIST+=(speech-dispatcher)
  # speech-dispatcher - Contains spd-say which is used by the service to announce status.
  # NOTE: speech-dispatcher forces the install of x11-common, which I'm not wild about for a Pi with no desktop,
  # BUT, I'm 99% sure that Slam Toolbox along with other ROS Navigation pieces installed later also depend on
  # x11-common, so removing this will not help.
  PACKAGE_TO_INSTALL_LIST+=(alsa-utils)
  # alsa-utils - Contains aplay, which is used by the service to play sounds.
fi

sudo apt install -y "${PACKAGE_TO_INSTALL_LIST[@]}"

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

printf "\n${YELLOW}[Cloning or Updating git repositories]${NC}\n"
cd ~/catkin_ws/src

printf "${LIGHTBLUE}ArloBot repository${NC}\n"
if ! [[ -d ~/catkin_ws/src/ArloBot ]]; then
  git clone -b noetic https://github.com/chrisl8/ArloBot.git
else
  cd ~/catkin_ws/src/ArloBot
  git checkout noetic
  git pull
fi

if [[ "${RESPONSE_TO_XV11_QUERY}" == "y" ]] || [[ ${CI} == "true" ]]; then # Always test in Travis
  printf "\n${LIGHTBLUE}Neato XV11 repository${NC}\n"
  # Only needed if you have an XV-11 "Neato" Scanner
  cd ~/catkin_ws/src
  if ! [[ -d ~/catkin_ws/src/xv_11_laser_driver ]]; then
    git clone -b noetic-devel https://github.com/chrisl8/xv_11_laser_driver.git
  else
    cd ~/catkin_ws/src/xv_11_laser_driver
    git pull
  fi
fi

if [[ "${RESPONSE_TO_SWEEP_QUERY}" == "y" ]] || [[ ${CI} == "true" ]]; then
  # Always test in Travis
  printf "\n${LIGHTBLUE}Scanse Sweep repository${NC}\n"
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
  cd ~/catkin_ws/src
  if ! [[ -d ~/catkin_ws/src/sweep-ros ]]; then
    # Using my repository to fix compile issues.
    git clone https://github.com/chrisl8/sweep-ros.git
  else
    cd ~/catkin_ws/src/sweep-ros
    git pull
  fi
fi

# TODO: Remove if they ever release freenect_stack for Noetic
if [[ "${RESPONSE_TO_KINECT_QUERY}" == "y" ]] || [[ ${CI} == "true" ]]; then
  # Always test in Travis
  printf "\n${LIGHTBLUE}OpenKinect for Kinect${NC}\n"
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
  cd ~/catkin_ws/src
  if ! [[ -d ~/catkin_ws/src/freenect_stack ]]; then
    git clone https://github.com/ros-drivers/freenect_stack.git
  else
    cd ~/catkin_ws/src/freenect_stack
    git pull
  fi
fi

if [[ "${RESPONSE_TO_RPLIDAR_QUERY}" == "y" ]] || [[ ${CI} == "true" ]]; then
  # Always test in Travis
  printf "\n${LIGHTBLUE}Slamtec RPLIDAR${NC}\n"
  cd ~/catkin_ws/src
  # NOTICE: The latest version entirely breaks functionality with slam_toolbox,
  # so I am switching to the last working commit.
  if ! [[ -d ~/catkin_ws/src/rplidar_ros ]]; then
    git clone https://github.com/Slamtec/rplidar_ros.git
    cd ~/catkin_ws/src/rplidar_ros
  else
    cd ~/catkin_ws/src/rplidar_ros
    #git pull # git pull does not work if you have checked out a specific commit, as we are currently doing.
  fi
  # NOTICE: The latest version entirely breaks functionality with slam_toolbox,
  # so I am switching to the last working commit.
  git checkout 4f899e670bec2c9e1f26b0969f2de36d23618ef3
fi

# TODO: Remove this if/when the apt package is released for noetic.
printf "\n${LIGHTBLUE}TEB Local Planner Tutorials${NC}\n"
printf "${LIGHT_PURPLE}The TEB Local Planner is installed via apt, but there is no noetic package for the tutorials.${NC}\n"
printf "${LIGHT_PURPLE}The tutorials are not required, but handy for reference.${NC}\n"
cd ~/catkin_ws/src
if ! [[ -d ~/catkin_ws/src/teb_local_planner_tutorials ]]; then
  git clone -b noetic-devel https://github.com/rst-tu-dortmund/teb_local_planner_tutorials.git
else
  cd ~/catkin_ws/src/teb_local_planner_tutorials
  git checkout noetic-devel
  git pull
fi

export PIP_DISABLE_PIP_VERSION_CHECK=1

if ! [[ ${WORKSTATION_INSTALL} == "y" ]]; then
  if [[ -d ~/catkin_ws/src/ArloBot/mycroft-core ]]; then
    printf "\n${YELLOW}[Updating Mycroft]${NC}\n"
    cd ~/catkin_ws/src/ArloBot/mycroft-core
    ./stop-mycroft.sh || true # Do not let failures crash the script.
    git pull
    ./dev_setup.sh
    ./start-mycroft.sh all
  else
    if [[ "${RESPONSE_TO_MYCROFT_QUERY}" == "y" ]]; then # Anything other than 'y' is NO (including null)
      printf "\n${YELLOW}[Installing Mycroft]${NC}\n"
      cd ~/catkin_ws/src/ArloBot
      git clone -b master https://github.com/MycroftAI/mycroft-core.git
      cd ~/catkin_ws/src/ArloBot/mycroft-core
      printf "\n${LIGHTBLUE}[There will be a lot of questions. I answer Yes to all of them personally.]${NC}\n"
      ./dev_setup.sh
      ./start-mycroft.sh all

      printf "\n${YELLOW}[IF you want to use Mycroft:]${NC}\n"
      printf "\n${YELLOW}[Then see https://docs.mycroft.ai/development/cerberus for configuration info.]${NC}\n"
      printf "\n${YELLOW}[See more info at: https://docs.mycroft.ai/installing.and.running/installation/git.clone.install]${NC}\n"
      printf "\n${YELLOW}[At the least you will have to register Mycroft if you want full functionality, although it does work without registering.]${NC}\n"
    fi
  fi

  if [[ -d /opt/mycroft/skills ]]; then
    printf "${LIGHTBLUE}Updating ArloBot Mycroft Skills${NC}\n"
    if ! [[ -L /opt/mycroft/skills/arlobot-robot-skill ]]; then
      cd /opt/mycroft/skills/
      ln -s "${HOME}/catkin_ws/src/ArloBot/mycroft-arlobot-skill" arlobot-robot-skill
    fi
    if ! [[ -L /opt/mycroft/skills/arlobot-smalltalk-skill ]]; then
      cd /opt/mycroft/skills/
      ln -s "${HOME}/catkin_ws/src/ArloBot/mycroft-smalltalk-skill" arlobot-smalltalk-skill
    fi
  fi
fi

cd ~/catkin_ws
printf "\n${YELLOW}[Installing dependencies for ROS build-from-source packages.]${NC}\n"
rosdep update
rosdep install -q -y -r --from-paths src --ignore-src
printf "\n${YELLOW}[(Re)Building ROS Source files.]${NC}\n"
catkin_make
# shellcheck source=/home/chrisl8/catkin_ws/devel/setup.bash
source ~/catkin_ws/devel/setup.bash
rospack profile

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
  if ! (grep "catkin_ws/devel/setup.bash" ~/.bashrc >/dev/null); then
    printf "\n${YELLOW}[Setting the ROS setup.bash source call in your .bashrc file]${NC}\n"
    sh -c "echo \"source ~/catkin_ws/devel/setup.bash\" >> ~/.bashrc"
  fi
  if ! (grep "${HOME}/catkin_ws/src/ArloBot/scripts" ~/.bashrc >/dev/null); then
    printf "\n${YELLOW}[Adding ArloBot Scripts folder to your path in .bashrc]${NC}\n"
    sh -c "echo \"export PATH=\\\$PATH:${HOME}/catkin_ws/src/ArloBot/scripts\" >> ~/.bashrc"
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
  if ! (grep "catkin_ws/devel/setup.zsh" ~/.zshrc >/dev/null); then
    printf "\n${YELLOW}[Setting the ROS setup.zsh source call in your .zshrc file]${NC}\n"
    sh -c "echo \"source ~/catkin_ws/devel/setup.zsh\" >> ~/.zshrc"
  fi
  if ! (grep "${HOME}/catkin_ws/src/ArloBot/scripts" ~/.zshrc >/dev/null); then
    printf "\n${YELLOW}[Adding ArloBot Scripts folder to your path in .zshrc]${NC}\n"
    sh -c "echo \"export PATH=\\\$PATH:${HOME}/catkin_ws/src/ArloBot/scripts\" >> ~/.zshrc"
  fi
fi

if ! [[ ${WORKSTATION_INSTALL} == "y" ]]; then
  # For 8-CH USB Relay board:
  pip3 install pylibftdi
  # Required by pylibftdi
  # https://pylibftdi.readthedocs.io/en/0.15.0/installation.html
  if ! [[ -f /etc/udev/rules.d/99-libftdi.rules ]]; then
    printf "\n${YELLOW}[Adding required sudo rule for pylibftdi to access USB based serial ports.]${NC}\n"
    sudo "${HOME}/catkin_ws/src/ArloBot/scripts/addRuleForUSBRelayBoard.sh"
    printf "${RED}You may have to reboot before the USB Relay board will function!${NC}\n"
  fi

  printf "\n${YELLOW}[Installing and Initializing the Current Node LTS version]${NC}\n"

  printf "${LIGHTBLUE}[Installing/Updating Node Version Manager]${NC}\n"
  if [[ -e ${HOME}/.nvm/nvm.sh ]]; then
    printf "${LIGHTBLUE}Deactivating existing Node Version Manager:${NC}\n"
    export NVM_DIR="${HOME}/.nvm"
    # shellcheck source=/home/chrisl8/.nvm/nvm.sh
    [[ -s "$NVM_DIR/nvm.sh" ]] && . "$NVM_DIR/nvm.sh" # This loads nvm
    nvm deactivate
  fi

  wget -qO- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.1/install.sh | bash
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
  printf "${LIGHTBLUE}You may get some errors here, that is normal. As long as things work, it is OK.$NC\n"
  cd
  printf "\n${YELLOW}[PM2 for running Robot service]$NC\n"
  npm install -g pm2
  printf "\n${YELLOW}[Log Streamer for Website]$NC\n"
  npm install -g log.io
  printf "\n${YELLOW}[Log.io File Watcher for Log.io Log Streamer]$NC\n"
  npm install -g log.io-file-input

  if [[ -d ~/catkin_ws/src/ArloBot/Log.io ]]; then
    printf "\n"
    printf "${LIGHTBLUE}Removing old Log.io Install${NC}\n"
    rm -rf "${HOME}/catkin_ws/src/ArloBot/Log.io"
  fi
fi

cd "${HOME}/catkin_ws/src/ArloBot/node"
printf "\n${YELLOW}[Grabbing node dependencies for scripts]${NC}\n"
printf "${LIGHTBLUE}You may get some errors here, that is normal. As long as things work, it is OK.$NC\n"
npm ci

if ! [[ ${WORKSTATION_INSTALL} == "y" ]]; then
  cd "${HOME}/catkin_ws/src/ArloBot/website"
  printf "\n${YELLOW}[Grabbing node dependencies for React website]${NC}\n"
  npm ci
  printf "\n${YELLOW}[Building React website]${NC}\n"
  npm run build

  if [[ "${ROS_META_PACKAGE}" == "desktop-full" ]]; then
    cd "${HOME}/catkin_ws/src/ArloBot/cypress-tests"
    printf "\n${YELLOW}[Installing Cypress.io for Tests]$NC\n"
    npm ci
  fi

  if ! (command -v mjpg_streamer >/dev/null); then
    printf "\n${YELLOW}[Installing mjpg_streamer for Web Page camera viewing]${NC}\n"
    cd "${HOME}/catkin_ws/src/ArloBot/"
    if ! [[ -d ${HOME}/catkin_ws/src/ArloBot/mjpg-streamer ]]; then
      git clone https://github.com/jacksonliam/mjpg-streamer.git
    else
      cd "${HOME}/catkin_ws/src/ArloBot/mjpg-streamer"
      git pull
    fi
    cd "${HOME}/catkin_ws/src/ArloBot/mjpg-streamer/mjpg-streamer-experimental"
    make distclean
    make
    sudo make install
    # Removing install files, as they are not needed anymore.
    cd "${HOME}/catkin_ws/src/ArloBot/"
    rm -rf mjpg-streamer
    # See scripts/streamVideoTest.sh for details on mjpg_streamer usage.
  fi
fi

if [[ -e ${ARLO_HOME}/personalDataForBehavior.json ]]; then
  node "${HOME}/catkin_ws/src/ArloBot/node/personalData.js"
else
  printf "\n"
  cp "${HOME}/catkin_ws/src/ArloBot/scripts/dotarlobot/personalDataForBehavior.json" "${ARLO_HOME}/"
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

if [[ ${architecture} == "x86_64" ]]; then

  if ! [[ -e /usr/share/PropWare/include/arlodrive.h ]]; then
    printf "\n${YELLOW}[Setting up PropWare and PropGCC for putting code on Activity Board.]${NC}\n"
    printf "${LIGHTBLUE}Parallax no longer supports Linux so we are using some third party tools.${NC}\n"
    printf "${LIGHTBLUE}https://david.zemon.name/PropWare${NC}\n"
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
    cp "${HOME}/catkin_ws/src/ArloBot/PropellerCodeForArloBot/dotfiles/per_robot_settings_for_propeller_c_code.h" "${ARLO_HOME}"
  fi

  if ! [[ -e ${ARLO_HOME}/per_robot_settings_for_propeller_2nd_board.h ]]; then
    cp "${HOME}/catkin_ws/src/ArloBot/PropellerCodeForArloBot/dotfiles/per_robot_settings_for_propeller_2nd_board.h" "${ARLO_HOME}"
  fi

  printf "\n${YELLOW}[Test Compiling Propeller Code.]${NC}\n"
  printf "${LIGHTBLUE}You will need to load this code onto your Propeller board after the setup is done.${NC}\n"

  function testMake() {
    printf " ${LIGHTBLUE}- ${1}${NC}\n"
    cd "${HOME}/catkin_ws/src/ArloBot/PropellerCodeForArloBot/${1}"
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

else
  printf "\n${YELLOW}NOTICE: Non-x86 systems CANNOT build the Propeller code!${NC}\n"
  printf "You will have to at least temporarily build run this setup on an x86 system in order\n"
  printf "to build the code for the Propeller board and load it.\n"
  printf "\nYou can use the 'Workstation' install if you don't want to put the entire package on that system.\n"
fi

if ! [[ ${WORKSTATION_INSTALL} == "y" ]]; then
  if [[ -e ${ARLO_HOME}/arlobot.yaml ]]; then
    if ! (diff "${HOME}/catkin_ws/src/ArloBot/arlobot_ros/param/arlobot.yaml" "${ARLO_HOME}/arlobot.yaml" >/dev/null); then
      printf "\n${GREEN}The ${RED}arlobot.yaml${GREEN} file in the repository is different from the one${NC}\n"
      printf "${GREEN}in your local settings.${NC}\n"
      printf "${GREEN}This is expected, but just in case, please look over the differences,${NC}\n"
      printf "${GREEN}and see if you need to copy in any new settings, or overwrite the file completely:${NC}\n"
      diff "${HOME}/catkin_ws/src/ArloBot/arlobot_ros/param/arlobot.yaml" "${ARLO_HOME}/arlobot.yaml" || true
      cp -i "${HOME}/catkin_ws/src/ArloBot/arlobot_ros/param/arlobot.yaml" "${ARLO_HOME}/"
      printf "\n"
    fi
  else
    printf "\n"
    cp "${HOME}/catkin_ws/src/ArloBot/arlobot_ros/param/arlobot.yaml" "${ARLO_HOME}/"
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

  if ! (crontab -l >/dev/null 2>&1) || ! (crontab -l | grep startpm2 >/dev/null 2>&1); then
    printf "\n${YELLOW}[Adding cron job to start web server on system reboot.]${NC}\n"
    # https://stackoverflow.com/questions/4880290/how-do-i-create-a-crontab-through-a-script
    (
      echo "@reboot ${HOME}/catkin_ws/src/ArloBot/startpm2.sh > ${HOME}/crontab.log"
    ) | crontab -
  fi

  printf "\n${LIGHT_PURPLE}[Flushing PM2 logs and starting/restarting web server.]${NC}\n"
  pm2 update
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

    # Special notices for the developer himself to keep his stuff up to date. =)
    cd "${HOME}/catkin_ws/src/ArloBot/node"
    printf "\n${RED}[Hey ${USER} please make sure the below items are up to date!]${NC}\n"
    printf "\n${GREEN}[Hey ${USER} please make sure the below items are up to date!]${NC}\n"
    printf "\n${LIGHT_PURPLE}[Hey ${USER} please make sure the below items are up to date!]${NC}\n"
    printf "${YELLOW}Does the current version of nvm we installed:${NC} "
    nvm --version
    printf "${YELLOW}Match the version on github:${NC} "
    wget -qO- https://github.com/creationix/nvm/blob/master/README.md | grep install.sh | grep wget | sed -e "s/<pre><code>//" | sed "s/\//\\n/g" | grep ^v | head -1
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
    cd "${HOME}/catkin_ws/src/ArloBot/website"
    npm outdated || true # Informational, do not crash  script
    printf "${YELLOW}in cypress tests/:${NC}\n"
    cd "${HOME}/catkin_ws/src/ArloBot/cypress-tests"
    npm outdated || true # Informational, do not crash  script
    printf "${LIGHT_PURPLE}-------------------------------------------------------${NC}\n"

    printf "${YELLOW}Diffing your param files to defaults where available:${NC}\n"
    printf "${LIGHTBLUE}teb_local_planner_params:${NC}\n"
    diff "${HOME}/catkin_ws/src/teb_local_planner_tutorials/cfg/diff_drive/teb_local_planner_params.yaml" "${HOME}/catkin_ws/src/ArloBot/arlobot_ros/param/teb_local_planner_params.yaml" || true
    printf "${LIGHTBLUE}twist_mux_locks:${NC}\n"
    diff /opt/ros/${INSTALLING_ROS_DISTRO}/share/twist_mux/config/twist_mux_locks.yaml "${HOME}/catkin_ws/src/ArloBot/arlobot_ros/param/twist_mux_locks.yaml" || true
    printf "${LIGHTBLUE}twist_mux_topics:${NC}\n"
    diff /opt/ros/${INSTALLING_ROS_DISTRO}/share/twist_mux/config/twist_mux_topics.yaml "${HOME}/catkin_ws/src/ArloBot/arlobot_ros/param/twist_mux_topics.yaml" || true
    printf "${LIGHTBLUE}mapper_params_localization:${NC}\n"
    diff "/opt/ros/${INSTALLING_ROS_DISTRO}/share/config/mapper_params_localization.yaml" "${HOME}/catkin_ws/src/ArloBot/arlobot_ros/param/mapper_params_localization.yaml" || true
    printf "${LIGHTBLUE}mapper_params_online_async:${NC}\n"
    diff "/opt/ros/${INSTALLING_ROS_DISTRO}/share/config/mapper_params_online_async.yaml" "${HOME}/catkin_ws/src/ArloBot/arlobot_ros/param/mapper_params_online_async.yaml" || true
  fi

  printf "\n${LIGHT_PURPLE}Anytime you want to update ArloBot code from the web you can run this same script again. It will pull down and compile new code without wiping out custom configs in ~/.arlobot. I run this script myself almost every day.${NC}\n"

  printf "\n${YELLOW}-----------------------------------${NC}\n"
  printf "${YELLOW}ALL DONE! REBOOT, EDIT FILES, AND START TESTING!${NC}\n\n"
  printf "${GREEN}Remember to edit the config files in ~/.arlobot${NC}\n\n"
  printf "${LIGHTCYAN}Go to ${LIGHTBLUE}http://$(node "${HOME}/catkin_ws/src/ArloBot/node/ipAddress.js"):$(jq '.webServerPort' "${ARLO_HOME}/personalDataForBehavior.json")${LIGHTCYAN} to see the Arlobot web interface.${NC}\n"
  printf "\n"
  printf "${GREEN}Look at README.md for testing ideas.${NC}\n"

  printf "\n${YELLOW}------------------------------------------------------------${NC}\n"
  printf "${YELLOW}Remember: You MUST install the Propeller code on your Propeller board too!${NC}\n"
  if ! [[ ${architecture} == "x86_64" ]]; then
    printf "HOWEVER, you must do this from an x86 system!\n"
  fi
  printf "${LIGHTBLUE}You can run install_Propeller_code.sh to perform the install,${NC}\n"
  printf "${LIGHTBLUE}and PropellerSerialTest.sh to test it.${NC}\n"
  printf "${GREEN}See: ${LIGHTBLUE}https://ekpyroticfrood.net/?p=551${NC}\n"
  printf "${GREEN}for more information on installing code on your Propeller board.${NC}\n"
  printf "${YELLOW}------------------------------------------------------------${NC}\n"
fi
# TODO: Some post install instructions for the workstation build.
INSTALL_FINISHED="true"
