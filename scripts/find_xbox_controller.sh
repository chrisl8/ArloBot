#!/usr/bin/env bash
# This will find the first controller,
# Increment FIND_NUMBER if you want to use another one
FIND_NUMBER=1
CONTROLLER_NUMBER=0
for i in $(ls /dev/input/js* 2>/dev/null); do
  #udevadm info -a -n $i|grep -m 1 product|grep Propeller > /dev/null
  if [[ $CONTROLLER_NUMBER -lt $FIND_NUMBER ]]; then
    udevadm info -a -n $i | egrep -m 1 name | grep "Xbox 360 Wireless Receiver" >/dev/null
    if [[ $? -eq 0 ]]; then
      echo $i
      CONTROLLER_NUMBER=$((${CONTROLLER_NUMBER} + 1))
    #echo ${CONTROLLER_NUMBER}
    fi
  fi
done
