#!/usr/bin/env bash
# shellcheck disable=SC2059 disable=SC2129
# ROS Melodic "Workstation" Automated Install
# This will set up a docker image to run via x11docker to use rviz
# on a secondary system. it will not run a robot.

# Run this straight off of github like this:
# bash <(wget -qO- --no-cache https://raw.githubusercontent.com/chrisl8/ArloBot/melodic/workstation-via-x11docker.sh)

set -e

BLUE='\033[0;34m'
RED='\033[0;31m'
LIGHT_PURPLE='\033[1;35m'
YELLOW='\033[1;33m'
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

if ! (command -v docker >/dev/null); then
  printf "${RED}[You must first install Docker to use this tool]${NC}\n"
  printf "${YELLOW}https://docs.docker.com/install/linux/docker-ce/ubuntu/${NC}\n"
  exit 1
fi

if ! (command -v xpra >/dev/null); then
  printf "\n${YELLOW}[Installing additional required Ubuntu Packages]${NC}\n"
  sudo apt install xpra # xpra lets you run the application in a resizable window!
fi

printf "\n${YELLOW}[Cloning or Updating ArloBot Code]${NC}\n"
if ! [[ -d ~/catkin_ws/src ]]; then
  mkdir -p ~/catkin_ws/src
fi
cd ~/catkin_ws/src
if ! [[ -d ~/catkin_ws/src/ArloBot ]]; then
  git clone -b melodic https://github.com/chrisl8/ArloBot.git
else
  cd ~/catkin_ws/src/ArloBot
  git pull
fi

printf "\n${YELLOW}[Cloning or Updating x11docker repository]${NC}\n"
cd ~/catkin_ws/src
if ! [[ -d ~/catkin_ws/src/x11docker ]]; then
  git clone https://github.com/mviereck/x11docker.git
else
  cd ~/catkin_ws/src/x11docker
  git pull
fi

if [[ -f ~/.bashrc ]]; then
  printf "\n${YELLOW}[Setting the ROS environment in your .bashrc file]${NC}\n"
  if ! (grep ROS_MASTER_URI ~/.bashrc >/dev/null); then
    read -rp "What is the host name or IP of your robot? " answer
    sh -c "echo \"export ROS_MASTER_URI=http://${answer}:11311\" >> ~/.bashrc"
  fi
  if ! (grep ROS_HOSTNAME ~/.bashrc >/dev/null); then
    sh -c "echo \"export ROS_HOSTNAME=$(uname -n).local\" >> ~/.bashrc"
  fi
  if ! (grep "${HOME}/catkin_ws/src/ArloBot/scripts" ~/.bashrc >/dev/null); then
    printf "\n${YELLOW}[Adding ArloBot Scripts folder to your path in .bashrc]${NC}\n"
    sh -c "echo \"export PATH=\\\$PATH:${HOME}/catkin_ws/src/ArloBot/scripts\" >> ~/.bashrc"
  fi
fi

if [[ -f ~/.zshrc ]]; then
  printf "\n${YELLOW}[Setting the ROS environment in your .zshrc file]${NC}\n"
  if ! (grep ROS_MASTER_URI ~/.zshrc >/dev/null); then
    # TODO: Use the answer from above if it exists, instead of asking twice.
    read -rp "What is the host name or IP of your robot? " answer
    sh -c "echo \"export ROS_MASTER_URI=http://${answer}:11311\" >> ~/.zshrc"
  fi
  if ! (grep ROS_HOSTNAME ~/.zshrc >/dev/null); then
    sh -c "echo \"export ROS_HOSTNAME=$(uname -n).local\" >> ~/.zshrc"
  fi
  if ! (grep "${HOME}/catkin_ws/src/ArloBot/scripts" ~/.zshrc >/dev/null); then
    printf "\n${YELLOW}[Adding ArloBot Scripts folder to your path in .zshrc]${NC}\n"
    sh -c "echo \"export PATH=\\\$PATH:${HOME}/catkin_ws/src/ArloBot/scripts\" >> ~/.zshrc"
  fi
fi

printf "\n${YELLOW}[Building Docker Image]${NC}\n"
cd ~/catkin_ws/src/ArloBot/docker-rviz/
sudo docker build -t ros:gui . # Run again if you change any settings

if ! (sudo -nl | grep x11docker >/dev/null); then
  printf "\n${YELLOW}[Setting up required sudo entries.]${NC}\n"
  echo "${USER} ALL = NOPASSWD: ${HOME}/catkin_ws/src/x11docker/x11docker" >/tmp/x11docker_sudoers
  chmod 0440 /tmp/x11docker_sudoers
  sudo chown root:root /tmp/x11docker_sudoers
  sudo mv /tmp/x11docker_sudoers /etc/sudoers.d/
  sudo chown root:root /etc/sudoers.d/x11docker_sudoers
fi

printf "\n${YELLOW}-----------------------------------${NC}\n"
printf "${YELLOW}ALL DONE! TRY RVIZ NOW${NC}\n"
printf "${LIGHT_PURPLE}If you have not set up Docker to run without root, you will have to use 'sudo' to run these.${NC}\n"
printf "${BLUE}Fire up your Arlobot on its machine, and then try these on here:${NC}\n"
printf "${BLUE} ~/catkin_ws/src/ArloBot/scripts/docker-view-navigation.sh${NC}\n"
printf "${BLUE} ~/catkin_ws/src/ArloBot/scripts/docker-view-robot.sh${NC}\n"
printf "\n"
printf "${BLUE}or run:${NC}\n"
printf "${BLUE} ~/catkin_ws/src/ArloBot/scripts/docker-ros-xterm.sh${NC}\n"
printf "${BLUE}     and run ROS commands by hand like:${NC}\n"
printf "${BLUE}              rqt_graph${NC}\n"
printf "\n"
INSTALL_FINISHED="true"
