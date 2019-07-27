#!/usr/bin/env bash
export PULSE_RUNTIME_PATH="/run/user/1000/pulse/"
if [[ $# -eq 0 ]]; then
  echo "You must provide a microphone name such as,"
  echo "PrimeSense or C615"
  echo "on the command line."
  exit
fi

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

DEVICE_NAME=$(${SCRIPTDIR}/find_microphone.sh ${1})

if [[ $? -eq 0 ]]; then
  /usr/bin/pacmd "set-default-source ${DEVICE_NAME}" &>/dev/null
else
  exit 1
fi
