cmake_minimum_required(VERSION 3.5)
project(arti_base_control)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV)
find_package(ament_cmake REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(arti_base_control_msgs REQUIRED)
find_package(angles REQUIRED)
find_package(dynamic_reconfigure REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/Axle.cfg
  cfg/BaseControl.cfg
  cfg/FourBarLinkageSteering.cfg
  cfg/IdealAckermannSteering.cfg
  cfg/Vehicle.cfg
)

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
  src/abstract_motor.cpp include/arti_base_control/abstract_motor.h)

ament_target_dependencies(${PROJECT_NAME}
  ${EIGEN3_LIBRARIES}
  ${OpenCV_LIBRARIES}
  "nav_msgs"
  "ackermann_msgs"
  "arti_base_control_msgs"
  "dynamic_reconfigure"
  "rclcpp"
  "pluginlib"
  "geometry_msgs"
  "tf2_ros"
  "angles"
  "sensor_msgs"
  "std_msgs"
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node
  src/base_control_node.cpp
)

ament_target_dependencies(${PROJECT_NAME}_node
  ${PROJECT_NAME}
)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

###########
## Build ##
###########

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -Wall -Wextra)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(rostest REQUIRED)
  target_link_libraries(test_wheel
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
  )

  # Wheel tests
  add_rostest_gtest(
    test_wheel
    test/test_wheel.test
    test/test_wheel.cpp
  )

endif()
