#!/bin/bash
# TODO: This is not system agnostic yet.
# Run 'arecord -L' to get the device list to pick from for the '-i' parameter.
# I use 'arecord -L|grep ^default' and pick one of those, but in theory you might not want a "default" card?
# This could be put into a config file later?
# I use this to start streaming audio from another system via SSH,
# then I run 'vlc rtp://234.5.5.5:1234' on that machine to listen.
avconv -f alsa -i default:CARD=C615 -acodec libmp3lame -ab 32k -ac 1 -re -f rtp rtp://234.5.5.5:1234 < /dev/null > /dev/null 2>/dev/null &
echo "Use 'pkill avconv' to stop this."
#
# The script I use on my remote machine to start this goes like this:
# $cat listenToArlobot.sh
#ssh arlobot 'pkill avconv'
#ssh arlobot '/home/chrisl8/catkin_ws/src/Arlobot/scripts/streamAudioInBackground.sh'
#vlc rtp://234.5.5.5:1234
#ssh arlobot 'pkill avconv'
#
# So it kills any existing instance, starts a new one, runs vlc to listen, and then kills it when done.
#
# If you are missing 'avconv' run 'sudo apt install libav-tools'
#
# If you are missing 'vlc" on your workstation run 'sudo apt install vlc-nox'
#

