#!/usr/bin/env bash

if (command -v jq >/dev/null) && [[ $(jq '.cloudServer.exists' "${HOME}/.arlobot/personalDataForBehavior.json") == true ]]; then
  if ! node "${HOME}/catkin_ws/src/ArloBot/node/addRobotIpToEtcHosts.js" >/dev/null; then
    echo "You will be asked for your password in order to have root access to edit /etc/hosts"
    echo "to add the host/ip for your robot."
    export NVM_DIR="$HOME/.nvm"
    # shellcheck source=/home/chrisl8/.nvm/nvm.sh
    [[ -s "$NVM_DIR/nvm.sh" ]] && \. "$NVM_DIR/nvm.sh" # This loads nvm
    sudo "${HOME}/.nvm/versions/node/$(nvm version)/bin/node" "${HOME}/catkin_ws/src/ArloBot/node/addRobotIpToEtcHosts.js"
  fi
fi
echo "set Global Options->Fixed Frame to"
echo "'odom' in order to make this work."
roslaunch arlobot_ros view_robot.launch --screen
