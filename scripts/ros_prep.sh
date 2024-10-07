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

if [[ ! -d ${HOME}/.arlobot/status/ ]]; then
  mkdir "${HOME}/.arlobot/status/"
fi

chmod 777 "${HOME}/.arlobot/status/" &>/dev/null

# ROS2 has no concept of "roscore", but we want to use a "blackboard" to hold parameters even if ROS isn't running.
# https://docs.ros.org/en/jazzy/Concepts/Basic/About-Parameters.html#migrating-from-ros-1
pkill -f parameter_blackboard
ros2 run demo_nodes_cpp parameter_blackboard &

while [[ $(ros2 param list) == '' ]]; do
  echo "Waiting for ROS parameter_blackboard to start . . ."
  sleep 1
done

ros2 param set /parameter_blackboard mapname empty

ros2 param set /parameter_blackboard maxPingRangeAccepted "$(jq '.maxPingRangeAccepted' "${HOME}/.arlobot/personalDataForBehavior.json")"

if [[ $(jq '.hasActivityBoard' "${HOME}/.arlobot/personalDataForBehavior.json") == true ]]; then
  ros2 param set /parameter_blackboard arlobot/port "$("${SCRIPTDIR}/find_ActivityBoard.sh")"
else
  YELLOW='\033[1;33m'
  NC='\033[0m' # NoColor
  printf "\n${YELLOW}Without an activity board your robot will not function!${NC}\n"
fi

if [[ $(jq '.hasXboxController' "${HOME}/.arlobot/personalDataForBehavior.json") == true ]]; then
  export HAS_XBOX_JOYSTICK=true
  #  /dev/input/js0 is the default is nothing is provided.
  if [[ $("${SCRIPTDIR}/find_xbox_controller.sh") ]]; then
    JOY_DEVICE="$("${SCRIPTDIR}/find_xbox_controller.sh")"
    export JOY_DEVICE
  fi
fi

if [[ $(jq '.camera0' "${HOME}/.arlobot/personalDataForBehavior.json") == true ]]; then
  CAMERA_NAME=$(jq '.camera0name' "${HOME}/.arlobot/personalDataForBehavior.json" | tr -d '"')
  ros2 param set /parameter_blackboard camera1 "$("${SCRIPTDIR}/find_camera.sh" "${CAMERA_NAME}")"
fi
if [[ $(jq '.camera1' "${HOME}/.arlobot/personalDataForBehavior.json") == true ]]; then
  CAMERA_NAME=$(jq '.camera1name' "${HOME}/.arlobot/personalDataForBehavior.json" | tr -d '"')
  ros2 param set /parameter_blackboard camera2 "$("${SCRIPTDIR}/find_camera.sh" "${CAMERA_NAME}")"
fi

# TODO: Add code to build the arlobot.urdf file if it doesn't exist and run colcon build again.

# TODO: Consider running colcon build every time this is started.

# TODO: Is there a "clean" option for colcon build?

