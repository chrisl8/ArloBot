[![Build Status](https://travis-ci.com/chrisl8/ArloBot.png)](https://travis-ci.com/chrisl8/ArloBot)
[![codecov](https://codecov.io/gh/chrisl8/ArloBot/branch/new-serial-interface/graph/badge.svg)](https://codecov.io/gh/chrisl8/ArloBot)

ArloBot Package for ROS
=======================

# THIS BRANCH IS STILL EXPERIMENTAL!!
### It is not fully tested yet, so use at your own risk and report issues.

## New Propeller C Code with new Serial Interface

* Entirely new Propeller C code. Based on original but with many changes
and modularized into .h files to help with organization.
Also using clangformat to format code.

* An entirely new serial interface has been written, using "binary" data
transmission that includes strict checks for length and checksums on
the data.

* An new python curses based serial communications test program now allows testing of ALL Propeller board functions over serial with zero use of ROS to more easily ensure the hardware is working before starting ROS.

## and SO MUCH MORE! ##

1. This package provides a set of ROS packages for using a [Parallax Arlo Platform](http://www.parallax.com/product/arlo-robotic-platform-system "Parallax") robot to run all of the demonstration projects for the [Robot Operating System (ROS)](http://www.ros.org/ "ROS") based [TurtleBot](http://wiki.ros.org/Robots/TurtleBot "TurtleBot")

2. This package also installs a node.js based front end GUI for the robot, complete with Twilio and Pushover integration and push button access to ROS functions:

### Mobile Friendly Web Interface  
![Alt text](/screenshots/arlobotNewWebInterface.png "Web Interface")

### New Curses Serial Testing Interface:  
`~/catkin_ws/src/ArloBot/scripts/PropellerSerialTest.sh`  
![Alt text](/screenshots/PropellerSerialTest.png "Serial Test Program")

An old demonstration video of this code on my ArloBot:
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
Arlobot operates on ROS Kinetic which requires Ubuntu *16.04 LTS*, or Xbuntu or Lubuntu of the same version. I personally use Lubuntu on my robot's on board computer.

If you put a fresh copy of Lubuntu 16.04 LTS on your robot's laptop then you can use the quick install script below.

## Quick Install: ##
There is now a script to install everything. Just run:

```
bash <(wget -qO- --no-cache https://raw.githubusercontent.com/chrisl8/ArloBot/new-serial-interface/setup-kinetic.sh)
```

Be sure to read the instructions that the script will print at the end about editing the config files in ~/.arlobot/

To update your code just run the same script again and it will pull down and compile anything new without erasing custom settings.

Please note that you will need the code to run on your Propeller board. This is stored in the "Propeller C Code for ArloBot" folder. Details on the Propeller code and setup are here: [http://ekpyroticfrood.net/?p=165](http://ekpyroticfrood.net/?p=165)

## Propeller Serial Interface Test ##
Before you start trying to get ROS running, but after you have loaded the C code onto the Propeller Activity board, use the PropellerSerialTest to test the hardware and interface.  

Place the robot up on blocks so it won't drive into anything if it goes nuts, and then run:  
`~/catkin_ws/src/ArloBot/scripts/PropellerSerialTest.sh`  
This provides an interface to send all controls, commands, and settings directly to the Propeller Activity Board without involving ROS. Use this to test everything and make sure your robot's hardware is functioning before you start playing with ROS.

#### Serial Test Usage
0. Make sure the robot is on blocks off of the floor so when the motors run it will stay still and not run into anything.  
1. On the robot run `~/catkin_ws/src/ArloBot/scripts/PropellerSerialTest.sh` It will not start moving or doing anything yet, but it may soon! 
2. The Proximity Sensors, that is the PING and/or InfraRed (IR) sensors, can cause the robot to move to avoid obstacles or refuse to move when commanded to. The bottom row of the status shows their measurements. The second from the bottom row shows if the Propeller Activity Board's built in safety code has determined if it is safe to move forward or backward or not at all.  There is also the line `Escaping:False` which indicates if the code is attempting to move to get away from something too close.  
Since we are on blocks, let's turn this off so it won't run the motors unless we tell it tell it to. This will prevent it from just driving the wheels in response to you or something close to your test setup.  
Pres `s` for Settings and then `a` to ignore All proximity sensors.  
The line next to `Settings` that says `ignoreAllProximitySensors:No` will change to `ignoreAllProximitySensors:Yes`.
3. Next note along the `Settings` line that `pluggedIn` is `Yes`. This will prevent the Propeller Activity Board code from sending any commands to the motor. Normally the only way to override this is through ROS. It is a safety measure that prevents the robot from ever moving if ROS is not running. We will override this now. If you left the Settings menu press `s` to get back in. Then `p` for Plugged in to turn that off.  `pluggedIn:Yes` will change to `pluggedIn:Yes` Now the robot can move.  
4. Now `q` to get out of Settings and `m` to send Move commands.  Use the letters `i` to to make the wheels move the robot forword.  
** IT WORKS!  **
You should probably test all of the functions available, but this has at least shown you how to use the test program and that your hardware is working.  You can use the Sensor outupt data to ensure your sensors work and diagnose issues with your hardware or code settings.  
There is also the ability to send Test packets to the robot to check for serious serial errors.  
Note that if you do the `r - Run speed test` there will be errors. It basically tests the ability for the code to slow down the transfer rate until the connection is stable, so errors will pop up as it attempts to go too fast and then backs off and retries. This is normal.  

## Quick Start of Entire Robot: ##
To start the Web Interface, which also allows starting ROS run:
```
~/catkin_ws/src/ArloBot/startRobot.sh
```
and point your web browser at the URL it gives you.

If you use Ubuntu or Lubuntu there should also be a desktop icon on the robot's desktop that you can run to do the same thing and bring up this web page on the robot itself.

## Workstation Install: ##
If you have a desktop or another laptop computer running Ubuntu that you just want to run
RVIZ, rqt_graph, etc. on, you can run this script to set up enough of ROS to do that, without
attempting to compile the robot code.  

**NOTE: These "workstation" installs will NOT run the robot! These are only for REMOTE work from another system.**

For Ubuntu Wily (15.10) and Xenial (16.04):
```
bash <(wget -qO- --no-cache https://raw.githubusercontent.com/chrisl8/ArloBot/new-serial-interface/workstation-kinetic.sh)
```  

For Ubuntu Zesty (17.04), Yakkety (16.10), and Xenial (16.04):  

```
bash <(wget -qO- --no-cache https://raw.githubusercontent.com/chrisl8/ArloBot/new-serial-interface/workstation-lunar.sh)
```  

For Ubuntu Cosmic (18.10):  

```
bash <(wget -qO- --no-cache https://raw.githubusercontent.com/chrisl8/ArloBot/new-serial-interface/workstation-melodic-on-cosmic.sh)
```  

and your system will be set up to use as a "remote" station.  

Once you have ROS running with a map loaded (or building) then you can run RVIZ on your worksattion.  
Look for the RVIZ icon on your desktop or in your Gnome Menu.  
If that isn't there you can run this script to start RVIZ on your workstation:  
`~/catkin_ws/src/ArloBot/scripts/view-navigation.sh`  
There are also other things you can do, like look at the Transform Graph:  
`~/catkin_ws/src/Arlobot/scripts/tf2pdf.sh`

To update your code just run the same script again and it will pull down and compile anything new without erasing custom settings.

## Full Arlobot Setup Instructions: ##
Complete setup and usage instructions are at my blog:
http://ekpyroticfrood.net/?p=162

## Edit your robot's Description ##
`roscd arlobot_description/urdf`  
and then read the Readme.txt file there!

## WARNING: BY DEFAULT YOUR ROBOT WILL TRY TO MOVE EVEN WHEN IT IS PLUGGED IN!!!!
Edit `~/.arlobot/arlobot.yaml`  
Set `monitorACconnection: True` to have ROS monitor the power connection and FREEZE  
the robot whenever the laptop is plugged into AC power  

If you want to disable AC connection monitoring in real time, while ROS is running, run `rosparam set /arlobot/monitorACconnection False`

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


## Convenience Scripts ##
Look in the scripts folder for a set of handy scripts for starting up and shutting down various aspects of Arlobot.


## HB-25 Motor Controller Support Gone! ##
Parallax has updated the Arlo platform to use their new DHB-10 Dual H-Bridge controller.  
My robot now uses the DHB-10 motor controller.  
Unfortunately I do not have time to support two controllers, especially when I only have one. (If I had money and time to build a second robot, maybe I could.)  
If you have HB-25 controllers, you can try using the last release that I made that still supported them here:  
[Old Propeller Code Release](https://github.com/chrisl8/ArloBot/releases/tag/oldPropellerCode)


## Contributing
All code contributions are greatly welcomed! I almost always accept pull requests. Worst case, I accept it, find an issue, and fix it, but even code that I have to fix up is better than code I have to write from scratch!  
Feel free to use this repository for [Hacktoberfest](https://hacktoberfest.digitalocean.com/) or other code contribution events, or just to get your feet wet using git. I'm happy to get spelling corrections and documentation improvements.  
I use [prettier](https://prettier.io/) on my JavaScript code and [Black](https://pypi.org/project/black/) on my Python code to format it. However, I won't let code formatting prevent me from accepting a pull request. I can tidy it up later.