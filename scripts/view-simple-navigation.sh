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

"${SCRIPTDIR}/addRobotIpToEtcHosts.sh"

echo "If you do not have a map loaded,"
echo "set Global Options->Fixed Frame to"
echo "'odom' in order to make this work."
# shellcheck source=/home/chrisl8/dev_ws/install/setup.bash
source ~/dev_ws/install/setup.bash
ros2 launch arlobot_ros view_simple_navigation.launch
