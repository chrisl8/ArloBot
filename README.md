ArloBot Package for ROS
===========================

This is my attempt to build a Package for the Robot Operating System (ROS) http://www.ros.org/ to work with the Parallax Propeller based ArloBot:
http://www.parallax.com/product/arlo-robotic-platform-system

Please note that you will also need the code to run on your Propeller board. This is stored in the Propeller folder.

## Basic usage instructions: ##
1. Build your ArloBot!
2. Get the SimpleIDE installed on a computer and load the code from the Propeller folder into your Activity Board. You can do this from Windows or Linux. I find it handy to have SimpleIDE running on my ROS laptop on board the Arlo though to make code updates easy.
3. Install Ubuntu on a laptop to ride on the ArloBot, and optionally also on a "workstation." I use a "Workstation" for anything that needs a GUI, and I use SSH to connect to the laptop and run anything that does not.
My "Workstation" is an Oracle VirtualBox installation of Ubuntu, which I find works great for RVIZ on my Windows desktop.
3. Install ROS Indigo on a Ubuntu laptop that can connect via USB to your Activity Board:
http://wiki.ros.org/indigo/Installation/Ubuntu
You will also need the Turtlebot packages, as I use that code when I can:
http://wiki.ros.org/turtlebot/Tutorials/indigo/Installation
NOTE: At this time Indigo Turtlebot has to be installed from source. If you have trouble doing that let me know and I will post my personal "how to" on installing Turtlebot from source to make this work.
4. Grab this code and put it on both your laptop and workstation:
```
cd
git clone git@github.com:chrisl8/ArloBot.git arlobot
cd ~/arlobot
source ~/turtlebot/devel/setup.bash
catkin_make
```
5. Set up your ROS environment. There are several commands that you have to run every time you start a terminal session to run ROS. What I do is put them into the .bashrc file in my home folder. I have these lines at the end of the .bashrc file on my "Workstation":
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

6. Depending on what you want to do there are different ways to "bring up" the robot.  These are the "recipes" that are well tested so far:

```
A good way to test your Propeller code is to run:
miniterm.py  /dev/ttyUSB1 115200
It will reset the Prop board and then start spitting out:
i 0
This tells you to initialize it, just send it:
d,0.403000,0.006760
and then it should start sending odometry info in the form of:
o       0.000   0.000   0.000   0.000   0.000   0.000   174
The last number is the distance from the front PING sensor.
If you want to get really fancy you can send it twist commands from the terminal

Basic TeleOp with 3D sensor use:
roslaunch arlobot_bringup minimal.launch
<New Terminal>
roslaunch arlobot_bringup keyboard_teleop.launch
<New Terminal>
roslaunch arlobot_bringup 3dsensor.launch
<GUI based Terminal>
roslaunch arlobot_bringup view_robot.launch
Tests from this setup:
  Set your Global Options->Fixed Frame to "odom
    Drive and see if the robot appears to move properly on the grid.
  Turn on LaserScan and set the Decay Time to 650
    Move around, spin in circles and see if you get a reasonable picture of the room.
  Turn off Laser Scan and turn on Registered DepthCloud to see if you get a picture of the room overlaied properly onto the 3D virtual world in RVIZ.
  

Gmapping Demo (SLAM Map building):
http://wiki.ros.org/turtlebot_navigation/Tutorials/Build%20a%20map%20with%20SLAM
roslaunch arlobot_bringup minimal.launch
<New Terminal>
roslaunch arlobot_bringup keyboard_teleop.launch
<New Terminal>
roslaunch arlobot_bringup gmapping_demo.launch
<GUI based Terminal>
roslaunch turtlebot_rviz_launchers view_navigation.launch
When you are done, save your map!
rosrun map_server map_saver -f ~/rosmaps/my_map1

AMCL (Navigating the map we built above:
http://wiki.ros.org/turtlebot_navigation/Tutorials/Autonomously%20navigate%20in%20a%20known%20map
roslaunch arlobot_bringup minimal.launch
<New Terminal>
roslaunch turtlebot_navigation amcl_demo.launch map_file:=~/rosmaps/my_map1.yaml
<GUI based Terminal>
roslaunch turtlebot_rviz_launchers view_navigation.launch --screen
NOTE: This is still in progress, it works with simple paths, but also seems quite willing to plow into a wall and spin its wheels desperately against a wall, even though the 3D camera should be telling it that it is smakc against a wall.
I need to see if there are settings to change for this,
see if thare are updates to Navigation in Indigo,
and set up my PING/IR sensors to provide Navigation input,
and code on the Propeller to provide override fail safe when the PING/IR detect obects that the Kinect/Xtion miss due to its height.
```
Please report an issue for any problems or if you need me to clarify anything!
