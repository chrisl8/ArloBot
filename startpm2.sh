#!/usr/bin/env bash
export NVM_DIR="${HOME}/.nvm"
# shellcheck source=/home/chrisl8/.nvm/nvm.sh
[ -s "$NVM_DIR/nvm.sh" ] && . "$NVM_DIR/nvm.sh" # This loads nvm

# USB Relay Controller
if [[ "$(jq '.useUSBrelay' "${HOME}"/.arlobot/personalDataForBehavior.json)" == true ]]; then
  echo "Turning off all relays"
  "${HOME}"/catkin_ws/src/ArloBot/scripts/switch_relay_name.sh all off
fi

MAX_RETRY_COUNT=10
RETRY_COUNT=0

# Master Power Relay
if [[ "$(jq '.useMasterPowerRelay' "${HOME}"/.arlobot/personalDataForBehavior.json)" == true ]]; then
  echo "Turning off Arlo Power."

  # It takes a while before it works after boot up.
  while ! "${HOME}"/catkin_ws/src/ArloBot/scripts/switch_master_relay.sh off && [[ ${RETRY_COUNT} -lt ${MAX_RETRY_COUNT} ]]; do
    echo ${RETRY_COUNT}
    sleep 1
    "${HOME}"/catkin_ws/src/ArloBot/scripts/switch_master_relay.sh off
    RETRY_COUNT=$((RETRY_COUNT + 1))
  done
  # The first one always results in some odd output and a failure, so do it one more time.
  sleep 1
  "${HOME}"/catkin_ws/src/ArloBot/scripts/switch_master_relay.sh off
fi

# NOTE: If you are using Mycroft, I suggest setting up Mycroft to start inside Xwindows,
# via the Startup Applications system.
# This seems to give it the best shot at using the correct audio input/output devices.

# Give the system a moment to finish booting before we come online.
# This can also give Mycroft time to start, so it speaks, if you are using it.
# Adjust as needed, or as you see fit.
sleep 30

cd "${HOME}"/catkin_ws/src/ArloBot/node/ || exit 1
"${HOME}"/.nvm/current/bin/pm2 start "${HOME}"/catkin_ws/src/ArloBot/node/pm2Config.json
