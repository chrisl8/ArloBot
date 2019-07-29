Convenience Scripts
===========================

These are scripts that are meant to make it easy to start up basic ROS functions on the robot without having to run dozens of roslaunch commands.  
Many of these scripts are also used by the web site and the ROS nodes themselves to accomplish tasks. Using the same code makes testing easy.

<ul>
<li>setup.sh - Set up everything Arlobot needs in your environment.</li>
<li>kill_ros.sh - This will stop all ROS programs and reset everything.</li>
<li>start-arlobot-only.sh - This will start up just the ArloBot basic ROS package</li>
<li>start-robot.sh - This will start up Arlobot, web interface and behavior, basically everything.</li>
<li>follow-object.sh - This will start the follow package and set the robot following objects.</li>
<li>auto-explore.sh - This will start up the autonomous exploration package and set the robot exploring! Be sure to save your map before killing this!</li>
<li>save-map.sh - Use this to save a map you've made before shutting down gmapping.</li>
<li>list-maps.sh - This will list the maps available in your maps folder.</li>
<li>load-map.sh - This will load up a map for your robot to start using.</li>
<li>view-navigation.sh - This will launch RVIZ with your robot, sensors and map all.</li>
<li>find_xbox_controller.sh - This will tell you what device your xBox 360 controller is on.</li>
<li>find_ActivityBoard.sh - This will tell you what USB port your Activity Board is on.</li>
<li>find_camera.sh - Given a camera "name" this will tell you what video device it is on.</li>
<li>find_QuickStart.sh - This will tell you what USB port an Parallax QuickStart board is on.</li>
<li>tf2pdf.sh - Run this in XWindows when ROS is running to bring up a transform map.</li>
<li>turn_off_motors.sh - This will turn off the motors using the USB Relay board.</li>
<li>turn_on_motors.sh - This will turn on the motors using the USB Relay board.</li>
<li>direct2PropSerialTest.sh - This is a shortcut to start directly listening to the Activity Board serial port.</li>
<li>check_hardware.sh - This is a helper script that other scripts call to check that all hardware is in place before starting.</li>
<li>drcontrol.py - This is the Python code from Sebastian Sjoholm to communicate with the USB Relay board.</li>
<li>model-robot.sh - This will just bring up enough of ROS to view a model of your robot.</li>
<li>ros_prep.sh - This is a helper script to get ports and things ready for ROS.</li>
<li>There are a lot more scripts in here that I have forgotten to document. Please bug me if you want me to update this.</li>
</ul>
