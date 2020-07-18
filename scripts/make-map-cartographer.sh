#!/usr/bin/env bash

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

# shellcheck source=/home/chrisl8/catkin_ws/src/ArloBot/scripts/rosEnvironmentSetup.sh
source "${SCRIPTDIR}/rosEnvironmentSetup.sh"

if pgrep -f robot.launch >/dev/null; then
  CARTOGRAPHER_DEVICE=$(jq '.cartographerMappingDevice' "${HOME}/.arlobot/personalDataForBehavior.json" | tr -d '"')
  if ! [[ "${CARTOGRAPHER_DEVICE}" == "none" ]]; then
    echo "When you are done, save your map!"
    echo "Please run './save-map.sh mapname' from another terminal when your map is done before closing this!"
    unbuffer roslaunch arlobot_cartographer "2d_cartograper_${CARTOGRAPHER_DEVICE}.launch"
  else
    echo "You must set your cartographerMappingDevice in ~/.arlobot/personalDataForBehavior.json first"
  fi
else
  echo "Robot must be running to start this."
fi
# 'unbuffer' is required for running this from the node based 'behavior'
# scripts. Otherwise stdout data is buffered until ROS exits,
# which makes monitoring status impossible.
# http://stackoverflow.com/a/11337310
# http://linux.die.net/man/1/unbuffer
