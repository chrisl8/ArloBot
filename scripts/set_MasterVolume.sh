#!/usr/bin/env bash
# Find the Audio device with a "Master" option
# in alsa and set it to the given percent.
if [[ $# -eq 0 ]]; then
  echo "You must provide a volume level."
  exit
fi
FOUND=1
for i in $(/usr/bin/aplay -l | grep card | awk '{ print $2 }' | tr -d ':' 2>/dev/null); do
  if ! [[ ${FOUND} == 0 ]] && amixer -c "${i}" scontrols 2>/dev/null | grep "Master" >/dev/null; then
    /usr/bin/amixer -c "${i}" set Speaker on >/dev/null
    /usr/bin/amixer -c "${i}" set Master "${1}%" on >/dev/null
    FOUND=0
  fi
done
exit ${FOUND}
