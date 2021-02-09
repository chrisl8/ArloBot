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

export NVM_DIR="${HOME}/.nvm"
# shellcheck source=/home/chrisl8/.nvm/nvm.sh
[[ -s "$NVM_DIR/nvm.sh" ]] && . "$NVM_DIR/nvm.sh" # This loads nvm

if ! (command -v log.io-server >/dev/null); then
  echo "You must install log.io-file-input to use this script. Run:"
  echo "npm install -g log.io"
  exit 1
fi

if ! (command -v log.io-file-input >/dev/null); then
  echo "You must install log.io-file-input to use this script. Run:"
  echo "npm install -g log.io-file-input"
  exit 1
fi

if [[ ! -d ${HOME}/.log.io ]]; then
  mkdir "${HOME}/.log.io"
fi

# Remove old config files if they exist.
if [[ -f ${HOME}/.log.io/harvester.conf ]]; then
  rm "${HOME}/.log.io/harvester.conf"
fi
if [[ -f ${HOME}/.log.io/log_server.conf ]]; then
  rm "${HOME}/.log.io/log_server.conf"
fi
if [[ -f ${HOME}/.log.io/web_server.conf ]]; then
  rm "${HOME}/.log.io/web_server.conf"
fi

echo "Creating config files for log.io..."
cp "${SCRIPTDIR}/dotarlobot/server.json" "${HOME}/.log.io/server.json"

if [[ ! -d ${HOME}/.log.io/inputs ]]; then
  mkdir -p "${HOME}/.log.io/inputs"
fi

INPUT_FILE=${HOME}/.log.io/inputs/file.json
jq -n '{"messageServer": {"host": "127.0.0.1", "port": 6689}}' >"${INPUT_FILE}"

# ROS Logs
logFolder=${HOME}/.ros/log
for j in "${HOME}"/.ros/log/latest/*; do
  logName="${j//.log/}"
  jq --arg logName "$(basename "${logName}")" --arg path "${j}" '.inputs += [{"source": $logName, "stream": "ROS Logs", "config": {"path": $path}}]' "${INPUT_FILE}" | sponge "${INPUT_FILE}"
done

# Grab any log files in the ~/.ros/log folder itself
logFolder=${HOME}/.ros/log
for j in $(find "${logFolder}" -maxdepth 1 -type f | sed 's#.*/##'); do
  logName="${j//.log/}"
  jq --arg logName "$(basename "${logName}")" --arg path "${j}" '.inputs += [{"source": $logName, "stream": "ROS Logs", "config": {"path": $path}}]' "${INPUT_FILE}" | sponge "${INPUT_FILE}"
done
echo "Starting server..."
"${HOME}/.nvm/current/bin/log.io-server" &
sleep 1 # A pause is required or the io-file-input program won't connect to the io-server
echo "Starting log file watcher..."
"${HOME}/.nvm/current/bin/log.io-file-input" &
sleep 2 # Pause to ensure the text below is listed BELOW the output of the above.
echo ""
echo "Set your browser to http://$(node "${SCRIPTDIR}/../node/ipAddress.js"):6688/ to watch logs"
echo "I find the Streams tab works best."
echo ""
echo "NOTE: This does not add new logs in real time,"
echo "so if new ROS nodes start up you will have to restart this."
echo ""
echo "To Stop log server run:"
echo "pkill -f log.io"
echo "Or kill_ros.sh also stops this."
