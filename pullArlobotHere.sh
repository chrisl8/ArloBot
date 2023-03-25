#!/usr/bin/env bash

# This script is part of my personal attempt to make Foxy work on this robot,
# but it doesn't work and probably never will.

WSL2_IP=172.30.224.167

# On a remote system with a fresh Ubuntu install run:

# cd;scp 172.30.224.167:/home/chrisl8/catkin_ws/src/ArloBot/pullArlobotHere.sh .;chmod +x pullArlobotHere.sh;./pullArlobotHere.sh;cd catkin_ws/src/ArloBot;./setup-noetic.sh

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

cd || exit

UNISON_ARGUMENTS=()
UNISON_ARGUMENTS+=(ssh://"${WSL2_IP}"/"${HOME}")
UNISON_ARGUMENTS+=("${HOME}")
UNISON_ARGUMENTS+=(-path catkin_ws/src/ArloBot)
UNISON_ARGUMENTS+=(-force ssh://"${WSL2_IP}"/"${HOME}")
UNISON_ARGUMENTS+=(-ignore "Name .idea")
UNISON_ARGUMENTS+=(-ignore "Name {*.pyc}")
UNISON_ARGUMENTS+=(-ignore "Name xscreen.png")
UNISON_ARGUMENTS+=(-ignore "Name xscreenOld.png")
UNISON_ARGUMENTS+=(-ignore "Name node_modules")
UNISON_ARGUMENTS+=(-ignore "Name mycroft-core")
UNISON_ARGUMENTS+=(-auto)

unison "${UNISON_ARGUMENTS[@]}"

cd || exit
