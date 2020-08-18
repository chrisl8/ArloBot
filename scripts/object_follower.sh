#!/usr/bin/env bash
# This will run the code from R. Patrick Goebel's excellent ROS By Example book
# http://www.lulu.com/shop/r-patrick-goebel/ros-by-example-indigo-volume-1/paperback/product-22094373.html
# to FOLLOW an oject around the room.
# Note that you must have Patrick's code cloned and imported into your system for this to work,
# and I suggest buying his book to learn how, although technically you could just get it from Github.

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

if pgrep -f robot.launch >/dev/null; then

  # shellcheck source=/home/chrisl8/catkin_ws/src/ArloBot/scripts/rosEnvironmentSetup.sh
  source "${SCRIPTDIR}/rosEnvironmentSetup.sh"

  export DISPLAY=:0
  unbuffer roslaunch arlobot_ros object_follower.launch --screen
else
  echo "Robot must be running to start this."
  exit 1
fi
