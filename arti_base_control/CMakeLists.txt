cmake_minimum_required(VERSION 3.5)
project(arti_base_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required ROS2 and system dependencies

## System dependencies are found with CMake's conventions
find_package(ament_cmake REQUIRED)

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(arti_base_control_msgs REQUIRED)
find_package(angles REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

###########
## Build ##
###########

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/axle.cpp include/arti_base_control/axle.h
  src/base_control.cpp include/arti_base_control/base_control.h
  src/velocity_controlled_joint_actuator.cpp include/arti_base_control/velocity_controlled_joint_actuator.h
  src/joint_limits.cpp include/arti_base_control/joint_limits.h
  src/joint_sensor.cpp include/arti_base_control/joint_sensor.h
  src/joint_state.cpp include/arti_base_control/joint_state.h
  src/joint_actuator_factory.cpp include/arti_base_control/joint_actuator_factory.h
  src/steering.cpp include/arti_base_control/steering.h
  src/position_controlled_joint_actuator.cpp include/arti_base_control/position_controlled_joint_actuator.h
  include/arti_base_control/types.h
  src/utils.cpp include/arti_base_control/utils.h
  src/vehicle.cpp include/arti_base_control/vehicle.h
  src/wheel.cpp include/arti_base_control/wheel.h
  src/abstract_motor.cpp include/arti_base_control/abstract_motor.h
)

target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}
  nav_msgs
  ackermann_msgs
  arti_base_control_msgs
  rclcpp
  pluginlib
  geometry_msgs
  tf2_ros
  angles
  sensor_msgs
  std_msgs
  tf2_geometry_msgs
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure

## Declare a C++ executable
add_executable(${PROJECT_NAME}_node
  src/base_control_node.cpp
)

target_include_directories(${PROJECT_NAME}_node PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
)

#############
## Install ##
#############

## install the library
install(TARGETS
    ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

## Mark cpp header files for installation
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
)

## Install the nodes
install(
  TARGETS
    ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  # launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # ROS TESTS (WITH LAUNCH)
  # ==========================================================================
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_ros REQUIRED)

  ament_add_gtest_executable(test_wheel test/test_wheel.cpp)
  target_link_libraries(test_wheel ${PROJECT_NAME})

  ament_add_ros_isolated_pytest_test(test_wheel_py "test/test_wheel.py"
    WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}")

endif()

##########################
## ament package export ##
##########################

ament_export_dependencies(
  nav_msgs
  ackermann_msgs
  arti_base_control_msgs
  rclcpp
  pluginlib
  geometry_msgs
  tf2_ros
  angles
  sensor_msgs
  std_msgs
  tf2_geometry_msgs
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()