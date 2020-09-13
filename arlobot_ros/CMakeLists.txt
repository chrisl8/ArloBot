cmake_minimum_required(VERSION 3.0.2)
project(arlobot_ros)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  geometry_msgs
  actionlib
  move_base_msgs
  joy
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  usbRelayStatus.msg
  arloStatus.msg
  arloSafety.msg
  arloButtons.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  ToggleRelay.srv
  ToggleLED.srv
  FindRelay.srv
  UnPlug.srv
  go_to_goal.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

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
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs
   joy
#  DEPENDS system_lib
)

###########
## Build ##
###########

add_executable(arlobot_teleop_joy src/turtlebot_joy.cpp)
target_link_libraries(arlobot_teleop_joy ${catkin_LIBRARIES})

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
