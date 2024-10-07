#!/usr/bin/env bash
# This will find the first controller,
# Increment FIND_NUMBER if you want to use another one
FIND_NUMBER=1
CONTROLLER_NUMBER=0
# Changes for Ubuntu 16.04:
# xpad driver no longer loads itself at boot.
# We no longer get the four "empty" js devices in /dev/input/ for js0 to js4 any time the wireless USB receiver is plugged in
# Instead /dev/input/js0 only appears the moment the actual wireless joystick is turned on!
if [[ -d /dev/input ]]; then
  # Looking for Xbox controller
  for i in /dev/input/js*; do
    #udevadm info -a -n $i|grep -m 1 product|grep Propeller > /dev/null
    if [[ $CONTROLLER_NUMBER -lt $FIND_NUMBER ]]; then
      if udevadm info -a -n "${i}" | grep -E -m 1 name | grep "Xbox 360 Wireless Receiver" >/dev/null; then
        echo "${i}"
        CONTROLLER_NUMBER=$((CONTROLLER_NUMBER + 1))
      fi
    fi
  done
else
  # No /dev/input directory found. This won't show up until you actually turn on the joystick. Sending a 'default'.
  echo "/dev/input/js0"
fi
