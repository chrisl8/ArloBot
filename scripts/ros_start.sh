#!/usr/bin/env bash
# This is the primary script to
# Start the entire robot

YELLOW='\033[1;33m'
LIGHT_PURPLE='\033[1;35m'
NC='\033[0m' # NoColor

DID_KILL=0

function finish() {
  if [ ${DID_KILL} -eq 0 ]; then
    DID_KILL=1
    ros_kill.sh
  fi
}
trap finish EXIT

printf "\n"
printf "${YELLOW}Starting Robot...${NC}\n"

# Grab and save the path to this script
# http://stackoverflow.com/a/246128
SOURCE="${BASH_SOURCE[0]}"
while [ -h "$SOURCE" ]; do # resolve $SOURCE until the file is no longer a symlink
  DIR="$(cd -P "$(dirname "$SOURCE")" && pwd)"
  SOURCE="$(readlink "$SOURCE")"
  [[ $SOURCE != /* ]] && SOURCE="$DIR/$SOURCE" # if $SOURCE was a relative symlink, we need to resolve it relative to the path where the symlink file was located
done
SCRIPTDIR="$(cd -P "$(dirname "$SOURCE")" && pwd)"
# echo "${SCRIPTDIR}" # For debugging

# shellcheck source=/home/chrisl8/ros2_ws/install/setup.bash
source "${HOME}/ros2_ws/install/setup.bash"

# shellcheck source=/home/chrisl8/dev_ws/src/ArloBot/scripts/ros_prep.sh
source "${SCRIPTDIR}/ros_prep.sh"

printf "${LIGHT_PURPLE}Use ros_kill.sh to close.${NC}\n"

# 'unbuffer' is required for running this from the node based 'behavior'
# scripts. Otherwise stdout data is buffered until ROS exits,
# which makes monitoring status impossible.
# http://stackoverflow.com/a/11337310
# http://linux.die.net/man/1/unbuffer
# Debug launch
#ros2 launch arlobot_ros robot_launch.py -d
# Normal launch
ros2 launch arlobot_ros robot_launch.py

# Then wrap up
finish
