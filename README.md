ArloBot Package for ROS
===========================

The goal of this project is to allow a Parallax Arlo Platform robot http://www.parallax.com/product/arlo-robotic-platform-system to run all of the demonstration projects for the Robot Operating System (ROS) based TurtleBot http://wiki.ros.org/Robots/TurtleBot

Because the Arlo robot is also a circular differential drive robot it behaves similarly to the Turtlebot. It just needs a set of C code to run on the Propeller based controller and a proper ROS node to communicate with it.

Here is a demonstration video of this code on my ArloBot:
http://youtu.be/7qJaA6K_WPE

## Quick Install: ##
There is now a script to install everything. Just run:

```
bash <(wget -qO- --no-cache https://raw.githubusercontent.com/chrisl8/ArloBot/master/setup.sh)
```
and you are done!

To update your code just run the same script again and it will pull down and compile anything new without erasing custom settings.

Please note that you will need the code to run on your Propeller board. This is stored in the "Propeller C Code for ArloBot" folder. Details on the Propeller code are here: http://ekpyroticfrood.net/?p=165

## Full Arlobot Setup Instructions: ##
Complete setup and usage instructions are at my blog:
http://ekpyroticfrood.net/?p=162

## Basic usage instructions: ##
Depending on what you want to do there are different ways to "bring up" the robot.<br/>These are the "recipes" that are well tested so far:
```
A good way to test your Propeller code is to run:
miniterm.py  /dev/ttyUSB0 115200
It will reset the Prop board and then start spitting out:
i 0
This tells you to initialize it, just send it:
d,0.403000,0.006760,0,0,0,0.0,0.0,0.0
and then it should start sending odometry info in the form of:
o       0.000   0.000   0.000   0.000   0.068   0.000   {"p0":135,"p1":90,"p2":78,"p3":78,"p4":107,"p5":34,"p6":15,"p7":11,"p8":16,"p9":67,"p10":77,"p11":120,"p12":9,"p13":10,"i0":1991,"i1":212,"i2":153,"i3":82,"i4":99,"i5":24,"i6":25,"i7":12}
With the occasional:
s       1       0       1       100     10      12      0.05    0.06    0
You may have fewer "p#" and "i#" instances. Those are the distance readings from your PING an IR sensors. They should change as you move around your robot.
The "s" line is handy because it tells you about some of the robots decisions. 1 = True and 0 = False
The meanings of each number are:
safeToProceed safeToRecede RobotIsEscaping ForwardSpeedLimit ReverseSpeedLimit SensorWithShortestDistanceReading LeftMotorPower RightMotorPower CliffDetected
If you want to get really fancy you can send it twist commands from the terminal too! Just remember to turn on the motors first for that to work!

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
