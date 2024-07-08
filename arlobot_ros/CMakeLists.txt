cmake_minimum_required(VERSION 3.5)
project(arlobot_ros)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

# Find dependencies
find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
# TODO: Do we need this?
#find_package(actionlib REQUIRED)
# TODO: How is this replaced in ROS2?
#find_package(move_base_msgs REQUIRED)
find_package(joy REQUIRED)
find_package(slam_toolbox REQUIRED)
# TODO: What do I do for this in ROS2?
#find_package(teb_local_planner REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

# Install Python executables
install(PROGRAMS
  scripts/propellerbot_node.py
  DESTINATION lib/${PROJECT_NAME}
)

## Generate messages in the 'msg' folder

## Generate services in the 'srv' folder

rosidl_generate_interfaces(
    ${PROJECT_NAME}
        msg/ArloSafety.msg
        msg/UsbRelayStatus.msg
        msg/ArloStatus.msg
        msg/ArloButtons.msg
        srv/UnPlug.srv
        srv/ToggleLED.srv
        srv/GoToGoal.srv
        srv/FindRelay.srv
        srv/ToggleRelay.srv
    DEPENDENCIES
        geometry_msgs
)
## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
# This is required to build ROS CPP packages in your own workspace
include_directories(
  
)

## Generate added messages and services with any dependencies listed here

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

###########
## Build ##
###########
ament_export_dependencies(rclpy std_msgs geometry_msgs joy slam_toolbox teb_local_planner)
ament_package()
