#!/usr/bin/env bash

# This is meant to remind/help us know how to create static transforms from the command line
# The online documentation is here, but it is lacking:
# http://wiki.ros.org/tf#static_transform_publisher

# First, you have to put "rosrun tf " in front of it.

# Second, there are two forms, and they are entirely devined from the number of parameters.
# Either:
#static_transform_publisher x y z yaw pitch roll frame_id child_frame_id period_in_ms

# or

#static_transform_publisher x y z qx qy qz qw frame_id child_frame_id  period_in_ms

# Note the use of "yaw pitch roll" or "qx qy qz qw"

# rpy in the URDF (xacro) files is for yaw, ptch, roll

# <!-- WARNING: The 'rpy' in the URDF (xacro) files means Roll, Pitch, Yaw, but in the static_transform_publisher the order -->
# <!-- is Yaw, Pitch, Roll !!!  So don't forget to translate those! -->

# Examples:
#ros2 run tf2_ros static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 1.0 base_link map
#ros2 run tf2_ros static_transform_publisher cam_px asus_xtion_pro_cam_py cam_pz cam_or cam_op cam_oy base_link camera_rgb_frame
#ros2 run tf2_ros static_transform_publisher 0.1085 0.0205 0.17 0.0 0.0 0.0 base_link camera_rgb_frame
#ros2 run tf2_ros static_transform_publisher 0.1285 0.01 0.17 0.0 0.0 0.0 1.0 base_link camera_rgb_frame
ros2 run tf2_ros static_transform_publisher 0.125 0.0 0.269107 3.1 0.0 0.0 base_link rplidar
