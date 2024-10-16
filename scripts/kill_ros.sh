#!/usr/bin/env bash
# Attempt to kill all aspects of ROS if it is already running

# Grab and save the path to this script
# http://stackoverflow.com/a/246128
SOURCE="${BASH_SOURCE[0]}"
while [ -h "$SOURCE" ]; do # resolve $SOURCE until the file is no longer a symlink
  DIR="$(cd -P "$(dirname "$SOURCE")" && pwd)"
  SOURCE="$(readlink "$SOURCE")"
  [[ $SOURCE != /* ]] && SOURCE="$DIR/$SOURCE" # if $SOURCE was a relative symlink, we need to resolve it relative to the path where the symlink file was located
done
SCRIPTDIR="$(cd -P "$(dirname "$SOURCE")" && pwd)"
# echo "${SCRIPTDIR}" # For debugging

echo "Killing everything, please wait..."
source "${HOME}/ros2_ws/install/setup.bash"
ros2 daemon stop
if (pkill -f log.io); then
  while (pkill -f log.io); do
    echo "Waiting for log.io to close . . ."
    sleep 1
  done
fi
if (pkill -f robot.launch); then
  while (pgrep -f robot.launch); do
    echo "Waiting for Robot to close . . ."
    sleep 1
  done
fi
if (pkill -f "arlobot_ros minimal.launch"); then
  while (pgrep -f "arlobot_ros minimal.launch"); do
    echo "Waiting for Arlobot to close . . ."
    sleep 1
  done
fi
if (pkill tf_echo); then
  while (pgrep tf_echo); do
    echo "Waiting for tf_echo to close . . ."
    sleep 1
  done
fi
if (pkill zbarcam); then
  while (pkill zbarcam); do
    echo "Waiting for zbarcam to close . . ."
    sleep 1
  done
fi
if (pkill -f ros2); then
  while (pgrep ros2); do
    echo "Waiting for ros2 to close . . ."
    sleep 1
  done
fi
if (pkill -f twist_mux); then
  while (pgrep -f twist_mux); do
    echo "Waiting for ROS twist_mux to close . . ."
    sleep 1
  done
fi
if (pkill -f parameter_blackboard); then
  while (pgrep -f parameter_blackboard); do
    echo "Waiting for ROS parameter_blackboard to close . . ."
    sleep 1
  done
fi
if (pkill -f static_transform_publisher); then
  while (pgrep -f static_transform_publisher); do
    echo "Waiting for static_transform_publisher to close . . ."
    sleep 1
  done
fi
if (pkill -f prime); then
  while (pgrep -f prime); do
    echo "Waiting for prime (3D camera) to close . . ."
    sleep 1
  done
fi
echo "Everything Killed."
if [[ -f nohup.out ]]; then
  rm nohup.out
fi

# Shut off XV11
if [[ $(jq '.hasXV11' "${HOME}/.arlobot/personalDataForBehavior.json") == true ]]; then
  echo "Stopping XV11..."
  "${SCRIPTDIR}/XVLidar.sh" stop
fi

# Shut off Scanse Sweep
if [[ $(jq '.hasScanseSweep' "${HOME}/.arlobot/personalDataForBehavior.json") == true ]]; then
  node "${HOME}/ArloBot/node/ScanseSweepControl.js" stop
fi

# USB Relay Controller
if [[ $(jq '.useUSBrelay' "${HOME}/.arlobot/personalDataForBehavior.json") == true ]]; then
  echo "Turning off all relays"
  "${SCRIPTDIR}/switch_relay_name.sh" all off
fi
