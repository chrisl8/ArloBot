# ArloBot Description

This folder holds the files that "describe" the physical
characteristics of the Arlobot.

To view the robot model run this from a GUI (Xwindows) session:
roslaunch arlobot_bringup model_robot.launch
or use the script:
~/catkin_ws/src/Metatron/scripts/model-robot.sh

Start with common.urdf.xacro which is what loads
the other files. It will help you see what each
file does.

There is also an Excel Spreadsheet in the root of this
repository called ArloURDF-Calculations.xlsx that has a lot of
measurements and calculations that I used to build
these files. You can use them to help with adjustments.

These files are based off of the files in:
roscd turtlebot_description/urdf/

