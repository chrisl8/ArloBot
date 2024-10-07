#!/usr/bin/env bash

if [[ -e ${HOME}/.nvm/nvm.sh ]]; then
  export NVM_DIR="$HOME/.nvm"
  # shellcheck source=/home/chrisl8/.nvm/nvm.sh
  [[ -s "$NVM_DIR/nvm.sh" ]] && . "$NVM_DIR/nvm.sh" # This loads nvm
fi
if [[ -e ${HOME}/.config/nvm/nvm.sh ]]; then
  export NVM_DIR="$HOME/.config/nvm"
  # shellcheck source=/home/chrisl8/.nvm/nvm.sh
  [[ -s "$NVM_DIR/nvm.sh" ]] && . "$NVM_DIR/nvm.sh" # This loads nvm
fi

# USB Relay Controller
if [[ "$(jq '.useUSBrelay' "${HOME}"/.arlobot/personalDataForBehavior.json)" == true ]]; then
  echo "Turning off all relays"
  "${HOME}"/dev_ws/src/ArloBot/scripts/switch_relay_name.sh all off
fi

# Give the system a moment to finish booting before we come online.
# Adjust as needed, or as you see fit.
# Even as little as 5 or 10 is probably fine, or pick a process to watch
# check does above.
sleep 15

cd "${HOME}"/dev_ws/src/ArloBot/node/ || exit 1
"${NVM_DIR}"/current/bin/pm2 start "${HOME}"/dev_ws/src/ArloBot/node/pm2Config.json
