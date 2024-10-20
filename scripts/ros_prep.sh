#!/usr/bin/env bash
# shellcheck disable=SC2059
# This script prepares the system to run ROS

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

if (pgrep -f simpleide >/dev/null); then
  echo "SimpleIDE is running,"
  echo "please close it and try again."
  exit 1
fi

if (ls "${HOME}"/.ros/log/* &>/dev/null); then
  echo "Clearing ROS Logs . . ."
  rm -r "${HOME}"/.ros/log/*
fi

# Check to make sure required hardware is present:
if ! ("${SCRIPTDIR}/check_hardware.sh"); then
  echo "ERROR: Hardware failed to come on line:"
  exit 1
fi

if [[ $(jq '.hasRPLIDAR' "${HOME}/.arlobot/personalDataForBehavior.json") == true ]]; then
  RPLIDAR_USB_PORT=$("${SCRIPTDIR}/find_RPLIDAR.sh")
  export RPLIDAR_USB_PORT
  RPLIDAR_BAUDRATE=$(jq '.rplidarBaudrate' "${HOME}/.arlobot/personalDataForBehavior.json")
  export RPLIDAR_BAUDRATE
fi

if [[ ! -d ${HOME}/.arlobot/status/ ]]; then
  mkdir "${HOME}/.arlobot/status/"
fi

chmod 777 "${HOME}/.arlobot/status/" &>/dev/null

if [[ $(jq '.hasActivityBoard' "${HOME}/.arlobot/personalDataForBehavior.json") == true ]]; then
  ACTIVITY_BOARD_PORT=$("${SCRIPTDIR}/find_ActivityBoard.sh")
  export ACTIVITY_BOARD_PORT
  maxPingRangeAccepted=$(jq '.maxPingRangeAccepted' "${HOME}/.arlobot/personalDataForBehavior.json")
  export maxPingRangeAccepted
  trackWidth=$(jq '.driveGeometry.trackWidth' "${HOME}/.arlobot/personalDataForBehavior.json")
  export trackWidth
  distancePerCount=$(jq '.driveGeometry.distancePerCount' "${HOME}/.arlobot/personalDataForBehavior.json")
  export distancePerCount
  wheelSymmetryError=$(jq '.driveGeometry.wheelSymmetryError' "${HOME}/.arlobot/personalDataForBehavior.json")
  export wheelSymmetryError
  ignoreProximity=$(jq '.ignoreProximity' "${HOME}/.arlobot/personalDataForBehavior.json")
  export ignoreProximity
  ignoreCliffSensors=$(jq '.ignoreCliffSensors' "${HOME}/.arlobot/personalDataForBehavior.json")
  export ignoreCliffSensors
  ignoreIRSensors=$(jq '.ignoreIRSensors' "${HOME}/.arlobot/personalDataForBehavior.json")
  export ignoreIRSensors
  ignoreFloorSensors=$(jq '.ignoreFloorSensors' "${HOME}/.arlobot/personalDataForBehavior.json")
  export ignoreFloorSensors
  pluggedIn=$(jq '.pluggedIn' "${HOME}/.arlobot/personalDataForBehavior.json")
  export pluggedIn
  activityBoardbaudRate=$(jq '.activityBoardbaudRate' "${HOME}/.arlobot/personalDataForBehavior.json")
  export activityBoardbaudRate
  lastX=0.0
  export lastX
  lastY=0.0
  export lastY
  lastHeading=0.0
  export lastHeading
else
  YELLOW='\033[1;33m'
  NC='\033[0m' # NoColor
  printf "\n${YELLOW}Without an activity board your robot will not function!${NC}\n"
fi
