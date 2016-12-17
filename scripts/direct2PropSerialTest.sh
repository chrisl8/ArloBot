#!/usr/bin/env bash
# This script helps to test the Propeller Activity Board
# by monitoring the serial port directly and suggesting
# commands that you can send to it.

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

if ! (which miniterm.py>/dev/null)
then
    sudo apt-get install python-serial
    echo "You may have to reboot before you can use the Propeller Board."
fi
if ! (id|grep dialout>/dev/null)
then
    sudo adduser ${USER} dialout
fi
echo "Activity Board should be sending a stead stream of:"
echo "i     0"
echo "i     0"
echo "etc."
echo "Paste this line into the terminal to get it to start sending sensor data:"
echo "For HB-25:"
echo "d,0.403000,0.006760,0,0,0,0,0,0.0,0.0,0.0"
echo "For DHB-10:"
echo "d,0.403000,0.00338,0,0,0,0,0,0.0,0.0,0.0"
miniterm.py $(${SCRIPTDIR}/find_ActivityBoard.sh) 115200

