#!/usr/bin/env bash
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

if [[ $# -lt 2 ]]
then
    echo "You must provide the relay name, and"
    echo "you must add 'on' or 'off' to the end of the command, like this:"
    echo "${0} fiveVolt on"
    echo ""
    echo "You can also include the USB Relay serial number on the end to speed things up, or if you have more than one."
    echo "${0} fiveVolt on A9026EI5"
    echo "Otherwise the script will find and use the first USB Relay."
    echo ""
    echo "You can use the relay name 'all' to switch all relays on or off, like this:"
    echo "${0} all off"
    exit
fi
if [ ${1} == 'all' ]
then
    RELAY_NUMBER=all
else
    HAS_RELAY_ENTRY="has_${1}"
    if [ $(jq ".relays.${HAS_RELAY_ENTRY}" ${HOME}/.arlobot/personalDataForBehavior.json) == true ]
    then
        RELAY_ENTRY="${1}"
        RELAY_NUMBER=$(jq ".relays.${RELAY_ENTRY}" ${HOME}/.arlobot/personalDataForBehavior.json)
    else
        exit
    fi
fi
if [[ $# -ne 3 ]]
then
    if [ -f ${TMPDIR}/usb_relay_serial_number ]
    then
        SERIAL_NUMBER=$(cat ${TMPDIR}/usb_relay_serial_number)
    else
       SERIAL_NUMBER=$(${SCRIPTDIR}/find_relay_serial_number.sh)
       if [[ $? != 0 ]]
       then
           exit 1
       else
           # Store the serial number to speed up future runs.
           # But use $TMPDIR so it goes away on reboot.
           echo ${SERIAL_NUMBER} > ${TMPDIR}/usb_relay_serial_number
       fi
    fi
else
    SERIAL_NUMBER=${3}
fi
${SCRIPTDIR}/drcontrol.py -d ${SERIAL_NUMBER} -r ${RELAY_NUMBER} -c ${2}
