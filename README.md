ArloBot Package for ROS
===========================

Here is a demonstration video of this code on my ArloBot:
http://youtu.be/7qJaA6K_WPE

This is my attempt to build a Package for the Robot Operating System (ROS) http://www.ros.org/ to work with the Parallax Propeller based ArloBot:
http://www.parallax.com/product/arlo-robotic-platform-system

Please note that you will also need the code to run on your Propeller board. This is stored in the "Propeller C Code for ArloBot" folder.

Please adjust settings for your robot in:

    ~/arlobot/src/arlobot/arlobot_bringup/param/arlobot.yaml

Before running this code!

## Basic usage instructions: ##
1. Build your ArloBot! http://ekpyroticfrood.net/?p=65
2. Install SimpleIDE on a computer and load the code from the "Propeller C Code for ArloBot" folder into your Activity Board's EEPROM. You can do this from Windows or Linux. I find it handy to have SimpleIDE running on my ROS laptop on board the Arlo to make code updates easy. http://learn.parallax.com/propeller-c-set-simpleide/linux
3. Install Ubuntu on a laptop to ride on the ArloBot, and optionally also on a "workstation." I use a "Workstation" for anything that needs a GUI, and I use SSH to connect to the laptop and run anything that does not.  My "Workstation" is an Oracle VirtualBox installation of Ubuntu, which I find works great for RVIZ on my Windows desktop.
4. Install ROS Indigo on a Ubuntu laptop that can connect via USB to your Activity Board: http://wiki.ros.org/indigo/Installation/Ubuntu
5. You will also need the Turtlebot packages, as I use that code when I can: http://wiki.ros.org/turtlebot/Tutorials/indigo/Installation
NOTE: At this time Indigo Turtlebot has to be installed from source. If you have trouble doing that let me know and I will post my personal "how to" on installing Turtlebot from source to make this work.
6. Grab this code and put it on both your laptop and workstation:
```
cd
git clone git@github.com:chrisl8/ArloBot.git arlobot
cd ~/arlobot
source ~/turtlebot/devel/setup.bash
catkin_make
```
7. Set up your ROS environment. There are several commands that you have to run every time you start a terminal session to run ROS. What I do is put them into the .bashrc file in my home folder. I have these lines at the end of the .bashrc file on my "Workstation":
```
export ROS_MASTER_URI=http://192.168.1.106:11311 # Set to laptop IP
export ROS_HOSTNAME=192.168.1.107 # Set to THIS machine's IP
export ROSLAUNCH_SSH_UNKNOWN=1
source ~/arlobot/devel/setup.bash
cd ~/arlobot/
```
and these lines at the end of the .bashrc on the laptop on the ArloBot:
```
export ROS_MASTER_URI=http://192.168.1.106:11311 # Set to laptop IP
export ROS_HOSTNAME=192.168.1.106 # Set to THIS machine's IP
export ROSLAUNCH_SSH_UNKNOWN=1
source ~/arlobot/devel/setup.bash
cd ~/arlobot/
```
Which makes it easy to just start running ROS commands as soon as I log in.

8. Depending on what you want to do there are different ways to "bring up" the robot.  These are the "recipes" that are well tested so far:

```
A good way to test your Propeller code is to run:
miniterm.py  /dev/ttyUSB1 115200
It will reset the Prop board and then start spitting out:
i 0
This tells you to initialize it, just send it:
d,0.403000,0.006760,0.0,0.0,0.0
and then it should start sending odometry info in the form of:
o       0.000   0.000   0.000   0.000   0.000   0.000   7       19      84      23      142     1249    227     13      143     60      9       7
The last 12 numbers are the distances from the PING and InfraRed sensors, they should change as you move around the ArloBot.
If you want to get really fancy you can send it twist commands from the terminal.

NOTE:
If you want to force arlobot_bringup to initialize the board even when the laptop
is plugged in run this after you start it to disable monitoring of AC power:
rosparam set /arlobot/monitorACconnection False
That can also be changed on arlobot.xml, but that command is useful for on the fly testing.

Basic TeleOp with 3D sensor use:
roslaunch arlobot_bringup minimal.launch --screen
<New Terminal>
roslaunch arlobot_teleop keyboard_teleop.launch
<New Terminal>
roslaunch turtlebot_bringup 3dsensor.launch
<GUI based Terminal>
roslaunch arlobot_rviz_launchers view_robot.launch
Tests from this setup:
  Set your Global Options->Fixed Frame to "odom
    Drive and see if the robot appears to move properly on the grid.
  Turn on LaserScan and set the Decay Time to 650
    Move around, spin in circles and see if you get a reasonable picture of the room.
  Turn off Laser Scan and turn on Registered DepthCloud to see if you get a picture of the room overlaied properly onto the 3D virtual world in RVIZ.
  

Gmapping Demo (SLAM Map building):
http://wiki.ros.org/turtlebot_navigation/Tutorials/Build%20a%20map%20with%20SLAM
roslaunch arlobot_bringup minimal.launch --screen
<New Terminal>
roslaunch arlobot_navigation gmapping_demo.launch --screen
<New Terminal>
roslaunch arlobot_teleop keyboard_teleop.launch
<GUI based Terminal>
roslaunch arlobot_rviz_launchers view_navigation.launch
When you are done, save your map!
rosrun map_server map_saver -f ~/rosmaps/my_map1
Tests from thsi setup:
  Make sure that obstacles in the Asus view are shown in the local costmap
    It is possible to map walls, while the 3D is ignored by the costmap!
    I find this is caused by the max_obstacle_height being set below the 3D Sensor's height
    in costmap_common_params.yaml on the "scan:" line

AMCL (Navigating the map we built above:
http://wiki.ros.org/turtlebot_navigation/Tutorials/Autonomously%20navigate%20in%20a%20known%20map
roslaunch arlobot_bringup minimal.launch --screen
<New Terminal>
roslaunch arlobot_navigation amcl_demo.launch map_file:=~/rosmaps/my_map1.yaml
<GUI based Terminal>
roslaunch arlobot_rviz_launchers view_navigation.launch --screen
NOTE: This is still in progress. It works pretty well, but needs a little tweaking.

Autonomous Map Making (Exploration):
roslaunch arlobot_bringup minimal.launch --screen
<New Terminal>
roslaunch arlobot_explore gmapping_explore.launch --screen
<New Terminal>
roslaunch hector_exploration_node exploration_planner.launch --screen
<GUI based Terminal>
roslaunch arlobot_rviz_launchers view_navigation.launch
<New Terminal>
roslaunch arlobot_explore arlobot_explore.launch --screen
When you are done, save your map!
rosrun map_server map_saver -f ~/rosmaps/my_map1

```
Please report an issue for any problems or if you need me to clarify anything!
