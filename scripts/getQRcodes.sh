#!/bin/bash
# Grab QR codes from webcam

# Grab and save the path to this script
# http://stackoverflow.com/a/246128
SOURCE="${BASH_SOURCE[0]}"
while [ -h "$SOURCE" ]; do # resolve $SOURCE until the file is no longer a symlink
  DIR="$( cd -P "$( dirname "$SOURCE" )" && pwd )"
  SOURCE="$(readlink "$SOURCE")"
  [[ $SOURCE != /* ]] && SOURCE="$DIR/$SOURCE" # if $SOURCE was a relative symlink, we need to resolve it relative to the path where the symlink file was located
done
SCRIPTDIR="$( cd -P "$( dirname "$SOURCE" )" && pwd )"
# echo ${SCRIPTDIR} # For debugging

if [ $(jq '.useQRcodes' ${HOME}/.arlobot/personalDataForBehavior.json) == true ]
    then
    if (pgrep -f zbarcam > /dev/null)
        then
        exit 1
    fi
    CAMERANAME=$(jq '.qrCameraName' ${HOME}/.arlobot/personalDataForBehavior.json | tr -d '"')
    VIDEODEVICE=$(${SCRIPTDIR}/find_camera.sh ${CAMERANAME})
    zbarcam -q --raw --nodisplay ${VIDEODEVICE}
fi
