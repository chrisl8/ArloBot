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

if [[ $# -eq 0 ]]; then
  echo "You must provide a map file name."
  echo "Run listMaps.sh for a list of your maps."
  exit
fi

export MAP_FILE_NAME=${HOME}/.arlobot/rosmaps/${1}

if [[ ! -f ${MAP_FILE_NAME}.data ]]; then
  echo "File ${MAP_FILE_NAME}.data does not exist!"
  exit 1
fi

if pgrep -f robot.launch >/dev/null; then
  # shellcheck source=/home/chrisl8/catkin_ws/src/ArloBot/scripts/rosEnvironmentSetup.sh
  source "${SCRIPTDIR}/rosEnvironmentSetup.sh"

  unbuffer roslaunch arlobot_ros slam_toolbox.launch
else
  echo "Robot must be running to start this."
  exit 1
fi

# 'unbuffer' is required for running this from the node based 'behavior'
# scripts. Otherwise stdout data is buffered until ROS exits,
# which makes monitoring status impossible.
# http://stackoverflow.com/a/11337310
# http://linux.die.net/man/1/unbuffer
