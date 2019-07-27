#!/usr/bin/env bash
if [[ $# -eq 0 ]]; then
  echo "You must provide a video device,"
  echo "such as /dev/video0 or /dev/video1 on the command line."
  exit
fi
echo "Go to http://${HOSTNAME}:58180 to see video stream."

# Grab and save the path to this script
# http://stackoverflow.com/a/246128
SOURCE="${BASH_SOURCE[0]}"
while [ -h "$SOURCE" ]; do # resolve $SOURCE until the file is no longer a symlink
  DIR="$(cd -P "$(dirname "$SOURCE")" && pwd)"
  SOURCE="$(readlink "$SOURCE")"
  [[ $SOURCE != /* ]] && SOURCE="$DIR/$SOURCE" # if $SOURCE was a relative symlink, we need to resolve it relative to the path where the symlink file was located
done
SCRIPTDIR="$(cd -P "$(dirname "$SOURCE")" && pwd)"
# echo ${SCRIPTDIR} # For debugging

mjpg_streamer -i "/usr/local/lib/input_uvc.so -d ${1} -f 30 -r 640x480" -o "/usr/local/lib/output_http.so -p 58180 -w ${SCRIPTDIR}/mjpg-streamer/mjpg-streamer/www"
