####################################################################################################
# Set minimum required version of cmake, project name and compile options                          #
####################################################################################################
cmake_minimum_required(VERSION 3.0.2)
project(room_seeker_core)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

####################################################################################################
# Find catkin packages and libraries for catkin and system dependencies                            #
####################################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf2_ros
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  room_seeker_msgs
)

####################################################################################################
# Setup for python modules and scripts                                                             #
####################################################################################################

####################################################################################################
# Declare ROS messages, services and actions                                                       #
####################################################################################################

####################################################################################################
# Declare ROS dynamic reconfigure parameters                                                       #
####################################################################################################

####################################################################################################
# catkin specific configuration                                                                    #
####################################################################################################
catkin_package(
  CATKIN_DEPENDS roscpp rospy tf2_ros std_msgs sensor_msgs geometry_msgs nav_msgs room_seeker_msgs
)

####################################################################################################
# Build                                                                                            #
####################################################################################################

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(room_seeker_core src/room_seeker_core.cpp src/room_seeker_level1.cpp)
add_dependencies(room_seeker_core ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(room_seeker_core ${catkin_LIBRARIES})

####################################################################################################
# Install                                                                                          #
####################################################################################################
## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  scripts/room_seeker_core_v1.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
install(TARGETS room_seeker_core
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

####################################################################################################
# Testing                                                                                          #
####################################################################################################
