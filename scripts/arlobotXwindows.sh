#!/bin/bash

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

export NVM_DIR="${HOME}/.nvm"
[ -s "$NVM_DIR/nvm.sh" ] && . "$NVM_DIR/nvm.sh"  # This loads nvm
nvm use stable
cd ${SCRIPTDIR}/../node
forever start --killSignal=SIGINT --minUptime=5000 --spinSleepTime=10000 --l="/tmp/arloBehavior.log" --o="/tmp/arloBehavior.log" --e="/tmp/arloBehavior.log" arloBehavior.js
forever list
firefox http://localhost:8080/localmenu.html
forever stop arloBehavior.js
rm "/tmp/arloBehavior.log"
