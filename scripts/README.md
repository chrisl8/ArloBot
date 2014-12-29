Convenience Scripts
===========================

These are scripts that are meant to make it easy to start up basic ROS functions on the robot without having to run dozens of roslaunch commands.

kill_ros.sh - This will stop all ROS programs and reset everything.
start-arlobot-only.sh - This will start up just the ArloBot basic ROS package
start-metatron.sh - This will start up Arlobot and Metatron, basically everything.
follow-object.sh - This will start the follow package and set the robot following objects.
auto-explore.sh - This will start up the autonomous exploration package and set the robot exploring.
save-map.sh - Use this to save a map you've made before shutting down gmapping.
list-maps.sh - This will list the maps available in your maps folder.
load-map.sh - This will load up a map for your robot to start using.
view-navigation.sh - This will launch RVIZ with your robot, sensors and map all included.
check_hardware.sh - This is a helper script that other scripts call to check that all hardware is in place before starting.
direct2PropSerialTest.sh - This is a shortcut to start directly listening to the Activity Board serial port.
drcontrol.py - This is the Python code from Sebastian Sjoholm to communicate with the USB Relay board.
find_ActivityBoard.sh - This will tell you what USB port your Activity Board is on.
find_camera.sh - Given a camera "name" this will tell you what video device it is on.
find_QuickStart.sh - This will tell you what USB port an Parallax QuickStart board is on.
find_xbox_controller.sh - This will tell you what device your xBox 360 controller is on.
model-robot.sh - This will just bring up enough of ROS to view a model of your robot.
view-model.sh - This will start RVIZ just to view the model-robot.
README.md
resetUSB.sh - This will reset all of your USB ports without rebooting.
ros_prep.sh - This is a helper script to get ports and things ready for ROS.
tf2pdf.sh
turn_off_motors.sh - This will turn off the motors using the USB Relay board.
turn_on_motors.sh - This will turn on the motors using the USB Relay board.
