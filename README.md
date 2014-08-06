ArloBot Package for ROS
===========================

This is my attempt to build a Package for the Robot Operating System (ROS) http://www.ros.org/ to work with the Parallax Propeller based ArloBot:
http://www.parallax.com/product/arlo-robotic-platform-system

Please note that you will also need the code to run on your Propeller board. This is stored in the Propeller folder. Hopefully catkin won't mind too much?

Basic usage instructions:

First build your ArloBot!
Second get the SimpleIDE installed on a computer and load the code from the Propeller folder into your Activity Board. You can do this from Windows or Linux. I find it handy to have SimpleIDE running on my ROS laptop on board the Arlo though to make code updates easy.

Third, install ROS Indigo on a Ubuntu laptop that can connect via USB to your Activity Board:
http://wiki.ros.org/indigo/Installation/Ubuntu
You will also need the Turtlebot packages, as I use that code when I can:
http://wiki.ros.org/turtlebot/Tutorials/indigo/Installation
NOTE: At this time Indigo Turtlebot has to be installed from source. If you have trouble doing that let me know and I will post my personal "how to" on installing Turtlebot from source to make this work.


Depending on what you want to do there are different ways to "bring up" the robot.  These are the "recipes" that are well tested so far:

NOTE: These all assume the basics are already set up in .bashrc for your robot!
```
A good way to test your Propeller code is to run:
miniterm.py  /dev/ttyUSB0 115200
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
roslaunch turtlebot_teleop keyboard_teleop.launch
<New Terminal>
roslaunch arlobot_bringup 3dsensor.launch
<GUI based Terminal>
roslaunch arlobot_bringup view_robot.launch
Tests from this setup:
  Set your Global Options->Fixed Frame to "odom
    Drive and see if the robot appears to move properly on the grid.
  Turn on LaserScan and set the Decay Time to 650
    Move around, spin in circles and see if you get a reasonable picture of the room.
  Turn Laser Scan and turn on Registered DepthCloud to see if you get a proper depth set picture of the room.
  

Gmapping Demo (SLAM Map building):
http://wiki.ros.org/turtlebot_navigation/Tutorials/Build%20a%20map%20with%20SLAM
roslaunch arlobot_bringup minimal.launch
<New Terminal>
roslaunch turtlebot_teleop keyboard_teleop.launch
<New Terminal>
roslaunch arlobot_bringup gmapping_demo.launch
<GUI based Terminal>
roslaunch turtlebot_rviz_launchers view_navigation.launch
When you are done, save your map!
rosrun map_server map_saver -f /tmp/my_map

AMCL (Navigating the map we built above:
http://wiki.ros.org/turtlebot_navigation/Tutorials/Autonomously%20navigate%20in%20a%20known%20map
roslaunch arlobot_bringup minimal.launch
<New Terminal>
roslaunch turtlebot_navigation amcl_demo.launch map_file:=/tmp/my_map.yaml
<GUI based Terminal>
roslaunch turtlebot_rviz_launchers view_navigation.launch --screen
NOTE: This is still in progress, it works with simple paths, but also seems quite willing to plow into a wall and spin its wheels desperately against a wall, even though the 3D camera should be telling it that it is smakc against a wall.
I need to see if there are settings to change for this,
see if thare are updates to Navigation in Indigo,
and set up my PING/IR sensors to provide Navigation input,
and code on the Propeller to provide override fail safe when the PING/IR detect obects within 6 inches.
```
Please report an issue for any problems or if you need me to clarify anything!
