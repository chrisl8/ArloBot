#!/usr/bin/env bash

rosrun tf2_tools view_frames.py
if evince frames.pdf 2>/dev/null; then
  rm frames.pdf
fi
rm frames.gv
