#!/usr/bin/env bash

# Grab and save the path to this script
# http://stackoverflow.com/a/246128
SOURCE="${BASH_SOURCE[0]}"
while [[ -L "$SOURCE" ]]; do # resolve $SOURCE until the file is no longer a symlink
  DIR="$(cd -P "$(dirname "$SOURCE")" && pwd)"
  SOURCE="$(readlink "$SOURCE")"
  [[ ${SOURCE} != /* ]] && SOURCE="$DIR/$SOURCE" # if $SOURCE was a relative symlink, we need to resolve it relative to the path where the symlink file was located
done
SCRIPT_DIR="$(cd -P "$(dirname "$SOURCE")" && pwd)"
#echo "${SCRIPT_DIR}" # For debugging

SCREEN_RESOLUTION=$(xdpyinfo | grep 'dimensions:' | awk '{print $2}')
if [[ "${SCREEN_RESOLUTION}" == "3840x2160" ]]; then
  "${SCRIPT_DIR}/../docker-rviz/call-via-x11docker.sh" "/catkin_ws/src/ArloBot/docker-rviz/xterm-large-font.sh"
else
  "${SCRIPT_DIR}/../docker-rviz/call-via-x11docker.sh" "xterm"
fi
