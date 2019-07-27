#!/usr/bin/env bash
# Launch Firefox with a certain page in X if it isn't already running:
if ! (pgrep firefox &>/dev/null); then
  export DISPLAY=:0
  nohup firefox "${1}" &>/dev/null &
fi
