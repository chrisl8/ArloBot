#!/bin/bash

# This will start up the motor on the XV11
# It must be spinning before ROS starts.

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

XV11PORT=$(${SCRIPTDIR}/find_XVLidar.sh)

setupPort () {
    stty -F ${XV11PORT} cs8 115200 ignbrk -brkint -icrnl -imaxbel -opost -onlcr -isig -icanon -iexten -echo -echoe -echok -echoctl -echoke noflsh -ixon -crtscts
}
if [ $? -eq 0 ]
    then
    case ${1} in
        'start')
            setupPort
            echo "ResetConfig" > ${XV11PORT}
            ;;
        'stop')
            setupPort
            echo "MotorOff" > ${XV11PORT}
            # Experience tells me one time is not enough.
            sleep 1
            setupPort
            echo "MotorOff" > ${XV11PORT}
            sleep 1
            setupPort
            echo "MotorOff" > ${XV11PORT}
            ;;
        'check')
            # Just run ONE 'stop' to check if it is ready to accept commands.
            setupPort
            echo "MotorOff" > ${XV11PORT}
            ;;
        *)
            echo 'Usage:'
            echo "${0} start - Start the XV 11 Lidar Motor"
            echo "${0} stop - Stop the XV 11 Lidar Motor"
            echo "${0} check - Check the XV 11 Lidar Motor"
            exit 1
            ;;
    esac

else
    echo "XV11 Not Found."
fi
