cmake_minimum_required(VERSION 3.5)
project(arti_base_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required ROS2 and system dependencies

## System dependencies are found with CMake's conventions
find_package(ament_cmake REQUIRED)

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(arti_base_control_msgs REQUIRED)
find_package(angles REQUIRED)
# find_package(dynamic_reconfigure REQUIRED) not used in ROS2
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

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

ament_target_dependencies(${PROJECT_NAME}  
  "nav_msgs"
  "ackermann_msgs"
  "arti_base_control_msgs"
  # "dynamic_reconfigure"
  "rclcpp"
  "pluginlib"
  "geometry_msgs"
  "tf2_ros"
  "angles"
  "sensor_msgs"
  "std_msgs"
  "tf2_geometry_msgs"
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure

## Declare a C++ executable
add_executable(${PROJECT_NAME}_node
  src/base_control_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME}_node rclcpp)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()

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

# Install targets to standard ROS2 destinations (lib/, include/, share/)

## Mark executables and/or libraries for installation
install(TARGETS
  ${PROJECT_NAME}
  ${PROJECT_NAME}_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
)

###########
## Build ##
###########

## Compile as C++14
add_compile_options(-std=c++14 -Wall -Wextra)

# Optionally install launch/config files
# install(DIRECTORY launch/ config/
#   DESTINATION share/${PROJECT_NAME}
# )

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_wheel test/test_wheel.cpp)
  if(TARGET test_wheel)
    target_link_libraries(test_wheel ${PROJECT_NAME})
    ament_target_dependencies(test_wheel rclcpp)
  endif()
endif()

