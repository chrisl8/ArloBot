#!/bin/bash
# This is meant to be called by stopROS.php
# and requires this line in /etc/sudoers:
# daemon ALL = (chrisl8) NOPASSWD: /home/chrisl8/arloweb/startROS.sh, /home/chrisl8/arloweb/stopROS.sh
# replacing "chrisl8" with the user id you run ROS as,
# and replacing "/home/chrisl8/arloweb" with the folder this is kept in.
# 
sudo -u $2 -g $2 ${1%/}/stopROS.sh

