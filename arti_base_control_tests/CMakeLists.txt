cmake_minimum_required(VERSION 3.5)
project(arti_base_control_tests)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

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

###########
## Build ##
###########

### --- Straight Driving Test Node ---
add_executable(straight_driving_test src/straight_driving_test.cpp)

target_include_directories(straight_driving_test PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

ament_target_dependencies(straight_driving_test
  rclcpp
  geometry_msgs
  nav_msgs
  tf2_ros
  arti_base_control
  ackermann_msgs
)

### --- Rotation Driving Test Node ---
add_executable(rotation_drive_test src/rotation_drive_test.cpp)

target_include_directories(rotation_drive_test PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

ament_target_dependencies(rotation_drive_test
  rclcpp
  geometry_msgs
  nav_msgs
  tf2_ros
  arti_base_control
  ackermann_msgs
)

#############
## Install ##
#############

## Install the nodes
install(
  TARGETS
    straight_driving_test
    rotation_drive_test
  DESTINATION lib/${PROJECT_NAME}
)

##########################
## ament package export ##
##########################

ament_package()