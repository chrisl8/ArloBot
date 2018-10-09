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
#
# This script is called by check_hardware.sh and direct2PropSerialTest.sh
# In order for it to work properly with direct2PropSerialTest.sh, it must ONLY write
# USB device information to the standard output (STDOUT).
# Therefore, in order to sucessfully search for two potential Propeller_Activity_[Brd|Board] types,
# the first search must only write to STDOUT if it is going to be successful.
#
# Prevent output to STDOUT and determine whether a "Propeller_Activity_Brd" will be found.
node ${SCRIPTDIR}/../node/UsbDevice.js "Propeller_Activity_Brd" ID_MODEL|grep USB &>/dev/null
if [ $? -eq 0 ]; then
  # "Propeller_Activity_Brd" will be found, so repeat command with result going to STDOUT.
  node ${SCRIPTDIR}/../node/UsbDevice.js "Propeller_Activity_Brd" ID_MODEL
else
  # "Propeller_Activity_Brd" will not be found, so try "Propeller_Activity_Board".
  # echo "Could not find 'Propeller_Activity_Brd', trying 'Propeller_Activity_Board'."
  node ${SCRIPTDIR}/../node/UsbDevice.js "Propeller_Activity_Board" ID_MODEL
fi
