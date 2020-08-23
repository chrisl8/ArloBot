#!/usr/bin/env bash
# Grab QR codes from webcam

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

if [[ $(jq '.useQRcodes' "${HOME}/.arlobot/personalDataForBehavior.json") == true ]]; then
  if (pkill zbarcam); then
    while (pkill zbarcam); do
      echo "Waiting for zbarcam to close . . ."
      sleep 1
    done
  fi
  CAMERA_NAME=$(jq '.qrCameraName' "${HOME}/.arlobot/personalDataForBehavior.json" | tr -d '"')
  if [[ -z ${CAMERA_NAME} ]]; then
    exit 1
  fi
  VIDEO_DEVICE=$("${SCRIPTDIR}/find_camera.sh" "${CAMERA_NAME}")
  if [[ -z ${VIDEO_DEVICE} ]]; then
    exit 1
  fi
  zbarcam -q --raw --nodisplay "${VIDEO_DEVICE}"
fi
