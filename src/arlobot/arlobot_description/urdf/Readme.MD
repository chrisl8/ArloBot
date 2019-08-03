# ArloBot Description

This folder holds the files that "describe" the physical
characteristics of the Arlobot.

To view the robot model run this from a GUI (Xwindows) session:
roslaunch arlobot_bringup model_robot.launch
or use the script:
~/catkin_ws/src/ArloBot/scripts/model-robot.sh

Start with common.urdf.xacro which is what loads
the other files. It will help you see what each
file does.

There is also an Excel Spreadsheet in the root of this
repository called ArloURDF-Calculations.xlsx that has a lot of
measurements and calculations that I used to build
these files. You can use them to help with adjustments.

These files are based off of the files in:
roscd turtlebot_description/urdf/

---

The robot's physical description is here in the .urdf.xacro files.

While a lot of this can be ignored as cosmetic, two things are important:

1. The physical footprint needs to be right. For arlobot though, that just means having one of the plates on board. As long as nothing sticks out from that plate, you are good.

2. The camera position on the robot is VERY IMPORTANT

There are two "sets" of files in here:
The 'default' or normal and the 'chrisl8' files.
I added the 'chrisl8' files so my constant robot changes wouldn't cause problems for everybody updating their data from github.

You an switch which files are loaded by the web site and scripts in  
`~/.arlobot/personalDataForBehavior.json`

You can even make your own personal set and load those.

The common_*.urdf.xacro file is what calls the other files.
The 3d_camera_position*.urdf.xacro is where you set the camera position.
The arlo*.urdf.xacro is where you set up all of the robot's characteristics.
The asus_xtion_pro*.urdf.xacro should generally not be edited. It is the xtion cameara setup. The one exception is that the cam_py left/right offset is set in there. If you camera is centered on your robot, then the given offset accounts for the camera being offset in the xtion itself. If your camera isn't centered left to right though, you may have to edit this.
NOTE: The camera IS offset, so be careful trying to get exact centers with markers. Because the camera is off center it can be confusing.

The way I calibrate the camera is this:

1. Set the robot on the floor with both axles exactly 1 meter from a solid wall. In other words, square to the wall. The axles are line up with the robot's exact center.

2. Start up gmapping, but don't move the robot at all.

3. Open up RVIZ and make sure you have the robot, the xtion data and the grid visible.
Look to see if the line that the xtion makes is on the 1 meter grid line in front of the robot.

If it needs to be brought closer or further from the robot adjust the cam_px value in 3d_camera_position_chrisl8.urdf.xacro

If the line is rotated in relation to the wall, adjust the cam_oy value.

You can also adjust the height of the camera with cam_pz, although this is less important, since the map is "2D".
