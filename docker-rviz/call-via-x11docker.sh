#!/usr/bin/env bash
# shellcheck disable=SC2059 disable=SC2129

set -e

# Grab and save the path to this script
# http://stackoverflow.com/a/246128
SOURCE="${BASH_SOURCE[0]}"
while [[ -L "$SOURCE" ]]; do # resolve $SOURCE until the file is no longer a symlink
  DIR="$(cd -P "$(dirname "$SOURCE")" && pwd)"
  SOURCE="$(readlink "$SOURCE")"
  [[ ${SOURCE} != /* ]] && SOURCE="$DIR/$SOURCE" # if $SOURCE was a relative symlink, we need to resolve it relative to the path where the symlink file was located
done
SCRIPT_DIR="$(cd -P "$(dirname "$SOURCE")" && pwd)"
#echo "${SCRIPT_DIR}" # For debugging

RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # NoColor

if ! (command -v docker >/dev/null); then
  printf "${RED}[You must first install Docker to use this tool]${NC}\n"
  printf "${YELLOW}https://docs.docker.com/install/linux/docker-ce/ubuntu/${NC}\n"
  exit 1
fi

if ! (command -v xpra >/dev/null); then
  printf "\n${YELLOW}[Installing additional required Ubuntu Packages]${NC}\n"
  sudo apt install xpra # xpra lets you run the application in a resizable window!
fi

if ! [[ -d ~/catkin_ws/src/x11docker ]]; then
  printf "${RED}[You must run workstation-melodic-via-x11docker.sh first to prepare your system for this tool]${NC}\n"
else
  cd ~/catkin_ws/src/x11docker
  git pull
fi

cd
DPI_ARGUMENT=""
SCREEN_RESOLUTION=$(xdpyinfo | grep 'dimensions:' | awk '{print $2}')
if [[ "${SCREEN_RESOLUTION}" == "3840x2160" ]]; then
  DPI_ARGUMENT=("--dpi" "200")
fi
VAR=$(node "${SCRIPT_DIR}/../node/ipAddress.js")
X11_DOCKER_ARGUMENT_LIST=("--xpra" "--hostnet" "${DPI_ARGUMENT[@]}" "--env" "ROS_MASTER_URI=${ROS_MASTER_URI}" "--env" "ROS_HOSTNAME=${VAR}" "ros:gui" "$@")

~/catkin_ws/src/x11docker/x11docker "${X11_DOCKER_ARGUMENT_LIST[@]}"
