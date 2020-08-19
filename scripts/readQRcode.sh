#!/usr/bin/env bash
# Read QR Code from video camera
# http://www.linux-magazine.com/Online/Features/Generating-QR-Codes-in-Linux
# Use qtrt to generate and print QR Codes with plain text for reading by installing:
# 'sudo apt install zbar-tools python-qrtools qtqr'
# on your workstation.
# Or just search the web for a QR Code generator.

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

if [[ $(jq '.camera0' "${HOME}/.arlobot/personalDataForBehavior.json") == true ]]; then
  CAMERANAME=$(jq '.camera0name' "${HOME}/.arlobot/personalDataForBehavior.json" | tr -d \")
  VIDEODEVICE=$("${SCRIPTDIR}/find_camera.sh" "${CAMERANAME}")
  #echo "You will have to kill this with Ctrl+c once you have the output you want."
  zbarcam -q --raw --nodisplay "${VIDEODEVICE}"
fi
