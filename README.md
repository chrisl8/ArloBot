ArloBot Package for ROS
===========================

This is my attempt to build a Package for the Robot Operating System (ROS) http://www.ros.org/ to work with the Parallax Propeller based ArloBot:
http://www.parallax.com/product/arlo-robotic-platform-system

Please note that you will also need the code to run on your Propeller board. This is stored in the Propeller folder. Hopefully catkin won't mind too much?

Basic usage instructions:

First build your ArloBot!
Second get the SimpleIDE installed on a computer and load the code from the Propeller folder into your Activity Board. You can do this from Windows or Linux. I find it handy to have SimpleIDE running on my ROS laptop on board the Arlo though to make code updates easy.

Third, install ROS Hydro on a Ubuntu laptop that can connect via USB to your Activity Board:
http://wiki.ros.org/hydro/Installation/Ubuntu
You will also need the Turtlebot packages, as I use that code when I can:
http://wiki.ros.org/turtlebot/Tutorials/hydro/Installation


Depending on what you want to do there are different ways to "bring up" the robot.  These are the "recipes" that are well tested so far:

NOTE: These all assume the basics are already set up in .bashrc for your robot!
```
Basic TeleOp with 3D sensor use:
roslaunch arlobot_bringup minimal.launch
<New Terminal>
roslaunch turtlebot_teleop keyboard_teleop.launch
<New Terminal>
roslaunch arlobot_bringup 3dsensor.launch
<GUI based Terminal>
roslaunch arlobot_bringup view_robot.launch

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
