cmake_minimum_required(VERSION 3.5)
project(arti_base_control_tests)

################################################
## Find required ROS 2 and system dependencies ##
################################################
find_package(ament_cmake REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(arti_base_control REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)

##############################################
## General build configuration and includes ##
##############################################
include_directories(
  include
)

#######################################
## Declare and build C++ executables ##
#######################################

### --- Straight Driving Test Node ---
add_executable(staight_driving_test src/staight_driving_test.cpp)
target_include_directories(staight_driving_test PUBLIC include)
ament_target_dependencies(staight_driving_test
  geometry_msgs
  rclcpp
)

### --- Rotation Driving Test Node ---
add_executable(rotation_drive_test src/rotation_drive_test.cpp)
target_include_directories(rotation_drive_test PUBLIC include)
ament_target_dependencies(rotation_drive_test
  rclcpp
  geometry_msgs
  nav_msgs
  tf2_ros
  arti_base_control
  ackermann_msgs
)

###################################
## Installation rules for ROS 2  ##
###################################
install(TARGETS
  staight_driving_test
  rotation_drive_test
  DESTINATION lib/${PROJECT_NAME}
)

##############################################
## Package declaration (replaces catkin_pkg) ##
##############################################
ament_package()
