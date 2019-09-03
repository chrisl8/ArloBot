#!/usr/bin/env bash
# Find the camera listed on the command line,
# based on output of 'pacmd'
export PULSE_RUNTIME_PATH="/run/user/1000/pulse/"
if [[ $# -eq 0 ]]; then
  echo "You must provide a microphone name such as,"
  echo "PrimeSense or C615"
  echo "on the command line."
  exit
fi
FOUND=1
for i in $(/usr/bin/pacmd list-sources | grep 'name:' | awk '{ print $2 }' 2>/dev/null); do
  if echo "${i}" | grep -i "${1}" >/dev/null; then
    echo "${i}" | tr -d '<' | tr -d '>'
    FOUND=0
  fi
done
exit ${FOUND}
