#!/usr/bin/env bash
if [[ $# -eq 0 ]]; then
  echo "You must provide a video device,"
  echo "such as /dev/video0 or /dev/video1 on the command line."
  echo
  echo "You can use ./find_camera.sh to get the device"
  exit
fi
echo "Go to http://${HOSTNAME}:58180 to see video stream."

# mjpg_streamer usage example:
#mjpg_streamer -i "/usr/local/lib/mjpg-streamer/input_uvc.so -n -d /dev/video0 -f 30 -r 640x480" -o "/usr/local/lib/mjpg-streamer/output_http.so -p 58180 -w /usr/local/share/mjpg-streamer/www"
# -n supresses the attempt to start the Pan/Tilt/Zoom (PTZ) controls.
# -r is the resolution, which you can adjust to reduce bandwidth usage.
# -f is the fremerate, which can also be adjusted to reduce bandwidth usage.
# -p is the port
# Then you can go to http://<robot-ip>:58180 to see the control panel with options to view and change camera settings.
# The direct video stream that the web site uses is http://<robot-ip>:58180/?action=stream
# Note that the entire point of mjpg_steamer is that it uses almost no resources on the sending system,
# but it is heavy on the network and receiving system.

# My Logitech c615 resolution is 1280 x 720 (or HD if you want but I don't want to eat the bandwidth)
# http://www.logitech.com/en-us/product/hd-webcam-c615

# Note that if you use the camera feature on the web interface, the frame rate and resolution are
# set in ~/.arlobot/personalDataForBehavior.json

mjpg_streamer -i "/usr/local/lib/mjpg-streamer/input_uvc.so -n -d ${1} -f 30 -r 1280x720" -o "/usr/local/lib/mjpg-streamer/output_http.so -p 58180 -w /usr/local/share/mjpg-streamer/www"
