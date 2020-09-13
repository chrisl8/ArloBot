#!/usr/bin/env bash
# Find the camera listed on the command line,
# based on output of 'fswebcam'
if [[ $# -eq 0 ]]; then
  echo "You must provide a camera name such as,"
  echo "HP or C615 or 'Integrated Camera'"
  echo "on the command line."
  exit
fi
FOUND=1
for i in /dev/video*; do
  if fswebcam --verbose --device="${i}" 2>&1 | grep cap.card | grep "${1}" >/dev/null; then
    # As of recent kernel revisions, Linux adds TWO /dev/video* interfaces for each camera!
    # One of them is just for extra metadata
    # https://unix.stackexchange.com/questions/512759/multiple-dev-video-for-one-physical-device
    # So we have to find the right one.
    if fswebcam --verbose --device="${i}" 2>&1 | grep -e 'YUYV' -e 'MJPG' >/dev/null; then
      echo "${i}"
      FOUND=0
    fi
  fi
done
exit ${FOUND}
