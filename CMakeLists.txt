cmake_minimum_required(VERSION 3.0.2)
project(astarplanning)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  std_msgs
)


# catkin_package(
#   INCLUDE_DIRS src/scripts
# #  LIBRARIES turtlebot_vacuum
# #  CATKIN_DEPENDS roscpp rospy std_msgs
# #  DEPENDS system_lib
# )

# catkin_install_python(PROGRAMS
# src/scripts/
# DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

catkin_package(
  INCLUDE_DIRS src/scripts
#  LIBRARIES planner
#  CATKIN_DEPENDS geometry_msgs rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)