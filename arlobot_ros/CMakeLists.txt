cmake_minimum_required(VERSION 3.5)
project(arlobot_ros)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
# TODO: How is this replaced in ROS2?
#find_package(move_base_msgs REQUIRED)
find_package(joy REQUIRED)
find_package(slam_toolbox REQUIRED)
# TODO: What do I do for this in ROS2?
#find_package(teb_local_planner REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/UsbRelayStatus.msg"
  "msg/ArloStatus.msg"
  "msg/ArloSafety.msg"
  "msg/ArloButtons.msg"
  "srv/ToggleRelay.srv"
  "srv/ToggleLED.srv"
  "srv/FindRelay.srv"
  "srv/UnPlug.srv"
  "srv/GoToGoal.srv"
  DEPENDENCIES geometry_msgs std_msgs
)

# Include Cpp "include" directory
include_directories(include)

# Create Cpp executable
add_executable(arlobot_teleop_joy src/turtlebot_joy.cpp)
ament_target_dependencies(arlobot_teleop_joy rclcpp)

# Install Cpp executables
install(TARGETS
  arlobot_teleop_joy
  DESTINATION lib/${PROJECT_NAME}
)
# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/propellerbot_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
