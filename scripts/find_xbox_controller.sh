#!/usr/bin/env bash
# This will find the first controller,
# Increment FIND_NUMBER if you want to use another one
FIND_NUMBER=1
CONTROLLER_NUMBER=0
for i in /dev/input/js*; do
  #udevadm info -a -n $i|grep -m 1 product|grep Propeller > /dev/null
  if [[ $CONTROLLER_NUMBER -lt $FIND_NUMBER ]]; then
    if udevadm info -a -n "${i}" | grep -E -m 1 name | grep "Xbox 360 Wireless Receiver" >/dev/null; then
      echo "${i}"
      CONTROLLER_NUMBER=$((CONTROLLER_NUMBER + 1))
    fi
  fi
done
