#!/usr/bin/env bash
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

if [[ $# -lt 1 ]]; then
  echo "You must add 'on', 'off' or 'read' to the end of the command, like this:"
  echo "${0} on"
  exit
fi

if ! (command -v node >/dev/null); then
  "${HOME}/.nvm/current/bin/node" "${SCRIPTDIR}/../node/MasterRelay.js" "${1}"
else
  node "${SCRIPTDIR}/../node/MasterRelay.js" "${1}"
fi
