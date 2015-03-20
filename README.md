ArloBot Package for ROS
===========================

Update February 24, 2015: I am still actively developing and using my "ArloBot", so if you have any questions or problems feel free to let me know!
The two initial goals for this package are:

1. To allow all of the functions of the TurtleBot from this tutorial: http://wiki.ros.org/Robots/TurtleBot to run on the "Arlo Platform". I believe that is possible now. If you find anything that doesn't work, open an issue!

2. I have purchased and am working though "ROS By Example INDIGO". All of the exmamples in this book should work on the Arlo Platform. As I'm working through the book now, I will update my code if I find anything that doesn't work properly. I highly recommend purchasing this book and working through it if you have or wish to build a ROS based robot! http://www.lulu.com/shop/r-patrick-goebel/ros-by-example-indigo-volume-1/ebook/product-22015937.html

3. Once these two goals are accomplished, the "ArloBot" becomes an open ended mobile development platform where anything is possible. I have a lot of ideas and it is just a matter of time and interest as to what specifically I will do. I have sevearl irons in the fire. This repository, however, will focus on goals 1 & 2 and I will put other more specialized functions into other repositories.

The goal of this project is to allow a Parallax Arlo Platform robot http://www.parallax.com/product/arlo-robotic-platform-system to run all of the demonstration projects for the Robot Operating System (ROS) based TurtleBot http://wiki.ros.org/Robots/TurtleBot

Because the Arlo robot is also a circular differential drive robot it behaves similarly to the Turtlebot. It just needs a set of C code to run on the Propeller based controller and a proper ROS node to communicate with it.

Here is a demonstration video of this code on my ArloBot:
http://youtu.be/7qJaA6K_WPE

Please note that you will need the code to run on your Propeller board. This is stored in the "Propeller C Code for ArloBot" folder.

Before running this code please adjust settings for your robot in:
    ~/arlobot/src/arlobot/arlobot_bringup/param/arlobot.yaml

## Arlobot Setup Instructions: ##
Complete setup and usage instructions are at my blog:
http://ekpyroticfrood.net/?p=162

## Basic usage instructions: ##
Depending on what you want to do there are different ways to "bring up" the robot.<br/>These are the "recipes" that are well tested so far:
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


Remote Control with an xBox 360 joystick:
http://ekpyroticfrood.net/?p=115
roslaunch arlobot_bringup minimal.launch --screen
<New Terminal>
rosparam set /joystick/dev "/dev/input/js0"
roslaunch turtlebot_teleop xbox360_teleop.launch --screen

Follow a person or object:
http://wiki.ros.org/turtlebot_follower/Tutorials/Demo
roslaunch arlobot_bringup minimal.launch --screen
<New Terminal>
roslaunch arlobot_bringup follower.launch --screen

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
roslaunch arlobot_explore exploration_planner.launch --screen
<GUI based Terminal>
roslaunch arlobot_rviz_launchers view_navigation.launch
<New Terminal>
roslaunch arlobot_explore arlobot_explore.launch --screen
When you are done, save your map!
rosrun map_server map_saver -f ~/rosmaps/my_map1

```
Please report an issue for any problems or if you need me to clarify anything!
