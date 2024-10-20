#!/usr/bin/env bash

GREEN='\033[0;32m'
NC='\033[0m' # NoColor

# Tweak these numbers if it keeps failing

# How long to delay between attempts
DELAY_BETWEEN_ATTEMPTS=2

# This script checks that all required hardware is present
# Grab and save the path to this script
# http://stackoverflow.com/a/246128
SOURCE="${BASH_SOURCE[0]}"
while [[ -L "$SOURCE" ]]; do # resolve $SOURCE until the file is no longer a symlink
  DIR="$(cd -P "$(dirname "$SOURCE")" && pwd)"
  SOURCE="$(readlink "$SOURCE")"
  [[ ${SOURCE} != /* ]] && SOURCE="$DIR/$SOURCE" # if $SOURCE was a relative symlink, we need to resolve it relative to the path where the symlink file was located
done
SCRIPT_DIR="$(cd -P "$(dirname "$SOURCE")" && pwd)"
# echo "${SCRIPT_DIR}" # For debugging

wrap_up_on_fail() {
  # USB Relay Controller
  if [[ $(jq '.useUSBrelay' "${HOME}/.arlobot/personalDataForBehavior.json") == true ]]; then
    echo "Turning off all relays"
    "${SCRIPT_DIR}/switch_relay_name.sh" all off
  fi
  exit 1
}

# USB Relay Controller
if [[ $(jq '.useUSBrelay' "${HOME}/.arlobot/personalDataForBehavior.json") == true ]]; then
  echo "Checking USB Relay Controller . . ."
  if ! "${SCRIPT_DIR}/drcontrol.py" -l | grep USB &>/dev/null; then
    echo "USB Relay Controller missing!"
    wrap_up_on_fail
  fi
fi

ATTEMPT_COUNT=1
CHECK_GOOD=false
FAILURE_REASON=""

check_hardware() {
  CHECK_GOOD=true
  echo "Checking all configured devices to make sure they are available, attempt ${ATTEMPT_COUNT}"
  ATTEMPT_COUNT=$((ATTEMPT_COUNT + 1))

  # Joystick
  # Changes for Ubuntu 16.04:
  # xpad driver no longer loads itself at boot.
  # We no longer get the four "empty" js devices in /dev/input/ for js0 to js4 any time the wireless USB receiver is plugged in
  # Instead /dev/input/js0 only appears the moment the actual wireless joystick is turned on!
  # NOTE: I tried xboxdrv and it was a nightmare. Unless I had the actual xbox controller powered on when I started it,
  # it would not recognize the controller and any time I power cycled the controller it changed device locations!
  if [[ $(jq '.hasXboxController' "${HOME}/.arlobot/personalDataForBehavior.json") == true ]]; then
    echo "Checking Xbox Controller . . ."
    # 1. Bring up the xpad driver after the USB power is on.
    # This doesn't hurt anything if it is already on.
    sudo modprobe xpad
  fi

  # Activity Board
  if [[ $(jq '.hasActivityBoard' "${HOME}/.arlobot/personalDataForBehavior.json") == true ]]; then
    echo "Checking Activity Board . . ."
    if ! "${SCRIPT_DIR}/find_ActivityBoard.sh" | grep USB &>/dev/null; then
      FAILURE_REASON="Activity Board missing! If this is a test install with no Activity Board, edit "${HOME}/.arlobot/personalDataForBehavior.json" and set 'hasActivityBoard' to false"
      CHECK_GOOD=false
      return 1
    fi

    # Quick Start Board
    # Assumption is that Quick Start Board will only be present with Activity Board.
    if [[ $(jq '.hasQuickStartBoard' "${HOME}/.arlobot/personalDataForBehavior.json") == true ]]; then
      echo "Checking Quick Start Board . . ."
      if ! "${SCRIPT_DIR}/find_QuickStart.sh" | grep USB &>/dev/null; then
        FAILURE_REASON="Quick Start Board missing!"
        CHECK_GOOD=false
        return 1
      fi
    fi
  fi

  # RPLIDAR
  if [[ $(jq '.hasRPLIDAR' "${HOME}/.arlobot/personalDataForBehavior.json") == true ]]; then
    echo "Checking RPLIDAR"
    if ! "${SCRIPT_DIR}/find_RPLIDAR.sh" | grep ttyUSB &>/dev/null; then
      FAILURE_REASON="RPLIDAR missing!"
      CHECK_GOOD=false
      return 1
    fi
  fi
}

check_hardware
while [[ ${CHECK_GOOD} == "false" && ATTEMPT_COUNT -lt RETRY_COUNT ]]; do
  echo "Giving devices ${DELAY_BETWEEN_ATTEMPTS} more seconds to come online..."
  sleep ${DELAY_BETWEEN_ATTEMPTS}
  check_hardware
done

if [[ ${CHECK_GOOD} == "false" ]]; then
  echo "${FAILURE_REASON}"
  wrap_up_on_fail
fi

printf "${GREEN}Hardware Check SUCCESS! All devices found.${NC}\n"
exit 0
