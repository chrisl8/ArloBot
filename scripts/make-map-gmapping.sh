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

if pgrep -f robot.launch; then
  echo "When you are done, save your map!"
  echo "Please run './save-map.sh mapname' from another terminal when your map is done before closing this!"
  if [[ $(jq '.use_xv11' "${HOME}/.arlobot/personalDataForBehavior.json") == true ]]; then
    unbuffer roslaunch arlobot_navigation gmapping_demo_xv11DWAonly.launch
  else
    unbuffer roslaunch arlobot_navigation gmapping_demo.launch
  fi
else
  echo "Robot must be running to start this."
fi
# 'unbuffer' is required for running this from the node based 'behavior'
# scripts. Otherwise stdout data is buffered until ROS exits,
# which makes monitoring status impossible.
# http://stackoverflow.com/a/11337310
# http://linux.die.net/man/1/unbuffer
