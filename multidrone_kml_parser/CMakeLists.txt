cmake_minimum_required(VERSION 2.8.3)
project(multidrone_kml_parser)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  multidrone_msgs
  roscpp
  rospy
  geodesy
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES multidrone_kml_parser
  CATKIN_DEPENDS multidrone_msgs roscpp rospy geodesy
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/multidrone_kml_parser.cpp src/pugixml.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
