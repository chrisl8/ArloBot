#!/usr/bin/env bash
# Attempt to kill all aspects of ROS if it is already running

YELLOW='\033[1;33m'
NC='\033[0m' # NoColor

printf "\n"
printf "${YELLOW}Shutting Down all ROS processes...${NC}\n"

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

if (pgrep -f rplidar > /dev/null); then
  # Try to ensure that the RPLidar is not left spinning
  source "${HOME}/ros2_ws/install/setup.bash"
  if ros2 service list | grep stop_motor > /dev/null; then
    echo "Stopping RPLidar . . ."
    timeout 10 ros2 service call /stop_motor std_srvs/srv/Empty
  fi
fi
if (pkill --ignore-ancestors -f log.io); then
  while (pgrep --ignore-ancestors -f log.io); do
    echo "Waiting for log.io to close . . ."
    sleep 1
    pkill --signal 9 --ignore-ancestors -f log.io
  done
fi
if (pkill --ignore-ancestors -f robot.launch); then
  while (pgrep --ignore-ancestors -f robot.launch); do
    echo "Waiting for Robot Launch to close . . ."
    sleep 1
    pkill --signal 9 --ignore-ancestors -f robot.launch
  done
fi
if pgrep --ignore-ancestors -f ros2cli.daemon > /dev/null; then
  echo "Stopping ROS2 daemon . . ."
  source "${HOME}/ros2_ws/install/setup.bash"
  timeout 20 ros2 daemon stop
fi
if (pkill --ignore-ancestors -f ros); then
  while (pgrep --ignore-ancestors -f ros); do
    echo "Waiting for leftover ROS nodes to close . . ."
    sleep 1
    pkill --signal 9 --ignore-ancestors -f ros
  done
fi
echo "Everything Killed."
if [[ -f nohup.out ]]; then
  rm nohup.out
fi
# USB Relay Controller
if [[ $(jq '.useUSBrelay' "${HOME}/.arlobot/personalDataForBehavior.json") == true ]]; then
  echo "Turning off all relays"
  "${SCRIPTDIR}/switch_relay_name.sh" all off
fi
