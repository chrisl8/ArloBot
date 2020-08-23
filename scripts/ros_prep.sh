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

# Set /scan topic source.
SCAN_TOPIC_SOURCE=$(jq -r '.scanTopicSource' "${HOME}/.arlobot/personalDataForBehavior.json")
export SCAN_TOPIC_SOURCE

# Set Active 3D Camera in CASE it is used.
ACTIVE_3D_CAMERA=$(jq -r '.active3dCamera' "${HOME}/.arlobot/personalDataForBehavior.json")
export ACTIVE_3D_CAMERA

if [[ $(jq '.hasASUSXtion' "${HOME}/.arlobot/personalDataForBehavior.json") == true ]]; then
  export HAS_ASUS_XTION=true
fi

if [[ $(jq '.hasKinect' "${HOME}/.arlobot/personalDataForBehavior.json") == true ]]; then
  export HAS_KINECT=true
fi

if [[ $(jq '.hasXV11' "${HOME}/.arlobot/personalDataForBehavior.json") == true ]]; then
  "${SCRIPTDIR}/XVLidar.sh" start
  export HAS_XV11=true
  XV11_SERIAL_PORT=$("${SCRIPTDIR}/find_XVLidar.sh")
  export XV11_SERIAL_PORT
fi

if [[ $(jq '.hasScanseSweep' "${HOME}/.arlobot/personalDataForBehavior.json") == true ]]; then
  export HAS_SCANSE_SWEEP=true
  SCANSE_SWEEP_SERIAL_PORT=$("${SCRIPTDIR}/find_ScanseSweep.sh")
  export SCANSE_SWEEP_SERIAL_PORT
fi

if [[ $(jq '.hasRPLIDAR' "${HOME}/.arlobot/personalDataForBehavior.json") == true ]]; then
  export HAS_RPLIDAR=true
  RPLIDAR_USB_PORT=$("${SCRIPTDIR}/find_RPLIDAR.sh")
  export RPLIDAR_USB_PORT
  RPLIDAR_BAUDRATE=$(jq '.rplidarBaudrate' "${HOME}/.arlobot/personalDataForBehavior.json")
  export RPLIDAR_BAUDRATE
else
  # Set to positive false because RPLIDAR defaults to true in robot.launch
  export HAS_RPLIDAR=false
fi

ARLOBOT_MODEL=$(jq '.arlobotModel' "${HOME}/.arlobot/personalDataForBehavior.json" | tr -d '"')
export ARLOBOT_MODEL

if [[ ! -d ${HOME}/.arlobot/status/ ]]; then
  mkdir "${HOME}/.arlobot/status/"
fi

chmod 777 "${HOME}/.arlobot/status/" &>/dev/null

if [[ ! -d ${HOME}/.arlobot/status/doors/ ]]; then
  mkdir "${HOME}/.arlobot/status/doors/"
fi

chmod 777 "${HOME}/.arlobot/status/doors/" &>/dev/null

# Start roscore separately so that we can set parameters
# before launching any ROS .launch files.
"/opt/ros/${ROS_DISTRO}/bin/roscore" &

while ! (rosparam list &>/dev/null); do
  echo "Waiting for roscore to start . . ."
  sleep 1
done

rosparam set /arlobot/mapname empty

rosparam set /arlobot/maxPingRangeAccepted "$(jq '.maxPingRangeAccepted' "${HOME}/.arlobot/personalDataForBehavior.json")"

if [[ $(jq '.hasActivityBoard' "${HOME}/.arlobot/personalDataForBehavior.json") == true ]]; then
  rosparam set /arlobot/port "$("${SCRIPTDIR}/find_ActivityBoard.sh")"
else
  YELLOW='\033[1;33m'
  NC='\033[0m' # NoColor
  printf "\n${YELLOW}Without an activity board your robot will not function!${NC}\n"
fi

if [[ $(jq '.hasXboxController' "${HOME}/.arlobot/personalDataForBehavior.json") == true ]]; then
  export HAS_XBOX_JOYSTICK=true
  if [[ $("${SCRIPTDIR}/find_xbox_controller.sh") ]]; then
    rosparam set /joystick/dev "$("${SCRIPTDIR}/find_xbox_controller.sh")"
  fi
fi

if [[ $(jq '.camera0' "${HOME}/.arlobot/personalDataForBehavior.json") == true ]]; then
  CAMERA_NAME=$(jq '.camera0name' "${HOME}/.arlobot/personalDataForBehavior.json" | tr -d '"')
  rosparam set /camera1 "$("${SCRIPTDIR}/find_camera.sh" "${CAMERA_NAME}")"
fi
if [[ $(jq '.camera1' "${HOME}/.arlobot/personalDataForBehavior.json") == true ]]; then
  CAMERA_NAME=$(jq '.camera1name' "${HOME}/.arlobot/personalDataForBehavior.json" | tr -d '"')
  rosparam set /camera2 "$("${SCRIPTDIR}/find_camera.sh" "${CAMERA_NAME}")"
fi

if [[ $(jq '.wait_for_door_confirmation' "${HOME}/.arlobot/personalDataForBehavior.json") == true ]]; then
  echo "Open and close each door to ensure lockout is working."
  for i in $(jq -r '.door_list[]' "${HOME}/.arlobot/personalDataForBehavior.json"); do
    touch "${HOME}/.arlobot/status/doors/${i}"
  done
  chmod ugo+rw "${HOME}"/.arlobot/status/doors/*
fi
