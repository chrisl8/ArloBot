#!/usr/bin/env bash

rosrun tf2_tools view_frames.py
evince frames.pdf 2>/dev/null
if [[ $? -eq 0 ]]; then
  rm frames.pdf
fi
rm frames.gv
