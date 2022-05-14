#!/usr/bin/env bash

# Grab and save the path to this script
# http://stackoverflow.com/a/246128
SOURCE="${BASH_SOURCE[0]}"
while [ -h "$SOURCE" ]; do # resolve $SOURCE until the file is no longer a symlink
  DIR="$(cd -P "$(dirname "$SOURCE")" && pwd)"
  SOURCE="$(readlink "$SOURCE")"
  [[ $SOURCE != /* ]] && SOURCE="$DIR/$SOURCE" # if $SOURCE was a relative symlink, we need to resolve it relative to the path where the symlink file was located
done
SCRIPT_DIR="$(cd -P "$(dirname "$SOURCE")" && pwd)"
# echo "${SCRIPT_DIR}" # For debugging

# shellcheck source=/home/chrisl8/catkin_ws/src/ArloBot/scripts/rosEnvironmentSetup.sh
source "${SCRIPT_DIR}/rosEnvironmentSetup.sh"

if pgrep -f robot.launch >/dev/null; then
  echo "Starting Auto Docking..."
  if ! "${SCRIPT_DIR}/find_docking_IR_Receiver.sh" >/dev/null; then
    echo "Docking IR Receiver missing!"
    exit 1
  fi
  export USB_PORT=$("${SCRIPT_DIR}/find_docking_IR_Receiver.sh")
  export ACQUISITION_ANGULAR_Z=$(jq '.dockingStation.acquisition_angular_z' "${HOME}/.arlobot/personalDataForBehavior.json" | tr -d '"')
  export APPROACH_ANGULAR_Z=$(jq '.dockingStation.approach_angular_z' "${HOME}/.arlobot/personalDataForBehavior.json" | tr -d '"')
  export LINEAR_X=$(jq '.dockingStation.linear_x' "${HOME}/.arlobot/personalDataForBehavior.json" | tr -d '"')

  # Ignore proximity sensors when docking
  rosparam set /arlobot/ignoreProximity true

  unbuffer roslaunch arlobot_ros auto_docking.launch --screen
  echo "Auto Docking closed." # For testing
else
  echo "Robot must be running to start this."
  exit 1
fi

# 'unbuffer' is required for running this from the node based 'behavior'
# scripts. Otherwise stdout data is buffered until ROS exits,
# which makes monitoring status impossible.
# http://stackoverflow.com/a/11337310
# http://linux.die.net/man/1/unbuffer
