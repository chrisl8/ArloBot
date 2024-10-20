# ArloBot Description

## Deploy

This is a `urdf` XML file with Macros in it. The macros are expanded by a tool called `xacro`.

You must use the `xacro` command to convert these files int a URDF file before running:  
` source ~/ros2_ws/install/setup.bash;/opt/ros/jazzy/bin/xacro ~/ArloBot/urdf/arlo.urdf.xacro > ~/ArloBot/arlobot_ros/arlobot.urdf`


The `setup-jazzy.sh` file does this automatically.

## Notes

This folder holds the files that "describe" the physical
characteristics of the Arlobot.

To view the robot model run this from a GUI session:
roslaunch arlobot_ros model_robot.launch
or use the script:
~/dev_ws/src/ArloBot/scripts/model-robot.sh

Start with common.urdf.xacro which is what loads
the other files. It will help you see what each
file does.

There is also an Excel Spreadsheet in the root of this
repository called ArloURDF-Calculations.xlsx that has a lot of
measurements and calculations that I used to build
these files. You can use them to help with adjustments.

---

The robot's physical description is here in the .urdf.xacro files.

While a lot of this can be ignored as cosmetic, two things are important:

1. The physical footprint needs to be right. For arlobot though, that just means having one of the plates on board. As long as nothing sticks out from that plate, you are good.

2. The lidar position on the robot is VERY IMPORTANT
