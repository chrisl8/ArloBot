cmake_minimum_required(VERSION 3.5)
project(arlobot_ros)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
  geometry_msgs
  actionlib
  move_base_msgs
  joy
  teb_local_planner
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  UsbRelayStatus.msg
  ArloStatus.msg
  ArloSafety.msg
  ArloButtons.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  ToggleRelay.srv
  ToggleLED.srv
  FindRelay.srv
  UnPlug.srv
  GoToGoal.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES arlobot_ros
  CATKIN_DEPENDS rospy std_msgs geometry_msgs
   joy slam_toolbox teb_local_planner
#  DEPENDS system_lib
)

###########
## Build ##
###########
add_executable(autodocking_irCode src/autodocking_irCode.cpp)
target_link_libraries(autodocking_irCode
   ${catkin_LIBRARIES}
)
## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
# This is required to build ROS CPP packages in your own workspace
include_directories(
  ${catkin_INCLUDE_DIRS}
)