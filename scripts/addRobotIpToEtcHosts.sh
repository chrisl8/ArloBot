#!/usr/bin/env bash

if (command -v jq >/dev/null) && [[ $(jq '.cloudServer.exists' "${HOME}/.arlobot/personalDataForBehavior.json") == true ]]; then
  export NVM_DIR="$HOME/.nvm"
  # shellcheck source=/home/chrisl8/.nvm/nvm.sh
  [[ -s "$NVM_DIR/nvm.sh" ]] && \. "$NVM_DIR/nvm.sh" # This loads nvm
  if ! node "${HOME}/catkin_ws/src/ArloBot/node/addRobotIpToEtcHosts.js" >/dev/null; then
    echo "You will be asked for your password in order to have root access to edit /etc/hosts"
    echo "to add the host/ip for your robot."
    # We must get the robot IP as the local user, because root has no knowledge of how to find the personalDataForBehavior.json file
    ROBOT_DATA=$(node "${HOME}/catkin_ws/src/ArloBot/node/getRobotDataFromWeb.js" json)
    export ROBOT_DATA
    sudo "${HOME}/.nvm/versions/node/$(nvm version)/bin/node" "${HOME}/catkin_ws/src/ArloBot/node/addRobotIpToEtcHosts.js" "${ROBOT_DATA}"
  fi
fi
