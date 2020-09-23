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

if [[ "$(jq '.useMyCroft' "${HOME}"/.arlobot/personalDataForBehavior.json)" == true ]]; then
  # My experience is that mycroft.client.enclosure comes up last,
  # and that once it is up, mycroft is ready to receive text.
  # This provides a faster startup (about 5 seconds) than just waiting for a static count of seconds.

  # The timeout ensures it starts anyway, especially after an initial install.
  MAX_WAIT_SECONDS=30
  WAITED_SECONDS=0
  while ! (pgrep -f mycroft.client.enclosure &>/dev/null)  && [[ ${WAITED_SECONDS} -lt ${MAX_WAIT_SECONDS} ]]; do
    echo "Waiting for up to ${MAX_WAIT_SECONDS} seconds for Mycroft to start. Waited ${WAITED_SECONDS} so far..."
    sleep 1
    WAITED_SECONDS=$((WAITED_SECONDS + 1))
  done
else
  # Give the system a moment to finish booting before we come online.
  # Adjust as needed, or as you see fit.
  # Even as little as 5 or 10 is probably fine, or pick a process to watch, like the mycroft
  # check does above.
  sleep 15
fi

cd "${HOME}"/catkin_ws/src/ArloBot/node/ || exit 1
"${HOME}"/.nvm/current/bin/pm2 start "${HOME}"/catkin_ws/src/ArloBot/node/pm2Config.json
