ArloBot Package for ROS
=======================

## and SO MUCH MORE! ##

1. This package provides a set of ROS packages for using a [Parallax Arlo Platform](http://www.parallax.com/product/arlo-robotic-platform-system "Parallax") robot to run all of the demonstration projects for the [Robot Operating System (ROS)](http://www.ros.org/ "ROS") based [TurtleBot](http://wiki.ros.org/Robots/TurtleBot "TurtleBot")

2. This package also installs a node.js based front end GUI for the robot, complete with Twilio and Pushover integration and push button access to ROS functions:

New Mobile friendly web layout:  
![Alt text](/screenshots/arlobotNewWebInterface.png "Web Interface")

Here is a demonstration video of this code on my ArloBot:
http://youtu.be/7qJaA6K_WPE

# Build a Robot! #
First you need to build a robot!

## Parts List ##
* Robot
* On board computer
* 3D Sensor

Unlike when I built my robot, the Arlo Robotic Platform now comes as a complete kit, which I recommend buying from Parallax: [Arlo Complete Robot System](https://www.parallax.com/product/28966)

You will also need a few other items (laptop and 3D sensor) which I have some details about on the [Parts List Wiki Page](https://github.com/chrisl8/ArloBot/wiki/Parts-List " Parts List")

## Building ##
Follow the excellent [Arlo Robot Assembly Guide](http://learn.parallax.com/tutorials/robot/arlo/arlo-robot-assembly-guide "Assembly Guide") at Parallax to both assemble and test your Arlo Robot platform.  
 Be sure you have fully understood and run all of their tests before moving on with using ROS.

Check out my blog: [ArloBot Build Index](http://ekpyroticfrood.net/?p=162 "Ekpyrotic Frood Blog")

And also jump on the [Parallax Forums](http://forums.parallax.com/ "Parallax Forums") and start searching and asking questions!

Now that the Arlo platform comes as a kit, things should be pretty straight forward, but we will all help you out with any problems you run into.  
Ask questions in the [Parallax Forums](http://forums.parallax.com/ "Parallax Forums"), on [GitHub](https://github.com/chrisl8/ArloBot/issues "Create an Issue"), on the [ROS for Arlobot Google Group](https://groups.google.com/forum/?utm_medium=email&utm_source=footer#!forum/ros-for-arlobot "ROS for Arlobot"), and on my [blog](http://ekpyroticfrood.net/ "My Blog").

Once your robot is built, you can use this package.

## Requirements ##
Arlobot operates on ROS Kinetic which requires Ubuntu 16.04 LTS, or Xbuntu or Lubuntu of the same version. I personally use Lubuntu on my robot's on board computer.

If you put a fresh copy of Lubuntu on your robot's laptop then you can use the quick install script below.

## Quick Install: ##
There is now a script to install everything. Just run:

```
bash <(wget -qO- --no-cache https://raw.githubusercontent.com/chrisl8/ArloBot/kinetic/setup-kinetic.sh)
```

Be sure to read the instructions that the script will print at the end about editing the config files in ~/.arlobot/

To update your code just run the same script again and it will pull down and compile anything new without erasing custom settings.

Please note that you will need the code to run on your Propeller board. This is stored in the "Propeller C Code for ArloBot" folder. Details on the Propeller code and setup are here: [http://ekpyroticfrood.net/?p=165](http://ekpyroticfrood.net/?p=165)

## Quick Start: ##
Run:
```
~/catkin_ws/src/ArloBot/startRobot.sh
```
and point your web browser at the URL it gives you.

If you use Ubuntu or Lubuntu there should also be a desktop icon on the robot's desktop that you can run to do the same thing and bring up this web page on the robot itself.

## Workstation Install: ##
If you have a desktop or another laptop computer running Ubuntu that you just want to run
RVIZ, rqt_graph, etc. on, you can run this script to set up enough of ROS to do that, without
attempting to compile the robot code.

```
bash <(wget -qO- --no-cache https://raw.githubusercontent.com/chrisl8/ArloBot/kinetic/workstation-kinetic.sh)
```  

or if you have a newer version of Ubuntu installed you can use ROS Lunar on your workstation (NOT the Robot!):  

```
bash <(wget -qO- --no-cache https://raw.githubusercontent.com/chrisl8/ArloBot/kinetic/workstation-lunar.sh)
```  

and your system will be set up to use as a "remote" station.  

Once you have ROS running with a map loaded (or building) then you can run RVIZ on your worksattion.  
Look for the RVIZ icon on your desktop or in your Gnome Menu.  
If that isn't there you can run this script to start RVIZ on your workstation:  
`~/catkin_ws/src/ArloBot/scripts/view-navigation.sh`  

To update your code just run the same script again and it will pull down and compile anything new without erasing custom settings.

## Full Arlobot Setup Instructions: ##
Complete setup and usage instructions are at my blog:
http://ekpyroticfrood.net/?p=162

## Edit your robot's Description ##
`roscd arlobot_description/urdf`  
and then read the Readme.txt file there!

## Serial Interface and Propeller Code Testing ##
It is a good idea to first test your serial interface to the propeller board:
```
cd ~/catkin_ws/src/ArloBot/scripts/
./direct2PropSerialTest.sh
```
This will make a direct serial connection to the Propeller Activity board.

It will reset the Prop board and then start spitting out:
```
i 0
```

Then paste the line suggested by the script to initialize the program
and it should start sending odometry info in the form of:
```
o       0.000   0.000   0.000   0.000   0.068   0.000   {"p0":135,"p1":90,"p2":78,"p3":78,"p4":107,"p5":34,"p6":15,"p7":11,"p8":16,"p9":67,"p10":77,"p11":120,"p12":9,"p13":10,"i0":1991,"i1":212,"i2":153,"i3":82,"i4":99,"i5":24,"i6":25,"i7":12}
```
With the occasional:
```
s       1       1       0       100     10      12      0.05    0.06    0    0
```
You may have fewer "p#" and "i#" instances. Those are the distance readings from your PING an IR sensors. They should change as you move around your robot.
The "s" line is handy because it tells you about some of the robot's decisions.

1 = True and 0 = False

The meanings of each numbers are:

- Safe to Proceed
- Safe to Recede
- Escaping
- Forward speed limit
- Reverse speed limit
- Distance sensor with lowest number
- Left motor voltage
- Right motor voltage
- Cliff detected
- Floor obstacle detected

If you want to get really fancy you can even send it twist commands from the terminal too! Just remember to turn on the motors first for that to work!

Twist command format:
```
s,0.0,0.0
```
where the first number is the linear meters per second and the second number is the angular radians per second of a standard ROS "Twist" message.

Slow Forward:
```
s,0.100,0.000
```
Slow Reverse:
```
s,-0.100,0.000
```
Slow turn left or right:
```
s,0.00,0.50
s,0.00,-0.50
```

## Basic ROS based usage instructions: ##
Depending on what you want to do there are different ways to "bring up" the robot with just ROS.<br/>These are the "recipes" that are well tested so far:

```
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
 Ask questions in the [Parallax Forums](http://forums.parallax.com/ "Parallax Forums"), on [GitHub](https://github.com/chrisl8/ArloBot/issues "Create an Issue"), on the [ROS for Arlobot Google Group](https://groups.google.com/forum/?utm_medium=email&utm_source=footer#!forum/ros-for-arlobot "ROS for Arlobot"), and on my [blog](http://ekpyroticfrood.net/ "My Blog"). We will write more documentation as we answer questions, and I hope you will also write instructions when you do your build!


##Convenience Scripts##
Look in the scripts folder for a set of handy scripts for starting up and shutting down various aspects of Arlobot.

## HB-25 Motor Controller Support untested! ##
Parallax has updated the Arlo platform to use their new DHB-10 Dual H-Bridge controller.  
My robot now uses the DHB-10 motor controller.  
In theory the HB-25 controller should continue to work with this code just fine, but as I no longer have them installed anywhere, I cannot test to be 100% sure I did not break something with them.  
If you do find a problem though, open an issue and I will try to fix it. I do not intend to end support for the HB-25, I just need your help to test it if it breaks.
