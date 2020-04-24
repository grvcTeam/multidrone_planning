cmake_minimum_required(VERSION 2.8.3)
project(global_tracker)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  geometry_msgs
  multidrone_msgs
  roscpp
  rospy
  tf
  multidrone_kml_parser
  std_msgs
  message_generation
  nav_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS sensor_msgs geometry_msgs multidrone_msgs roscpp rospy tf multidrone_kml_parser std_msgs message_runtime
)


include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(global_tracker src/global_tracker_node.cpp src/global_tracker.cpp)
target_link_libraries(global_tracker ${catkin_LIBRARIES})
add_dependencies(global_tracker ${catkin_EXPORTED_TARGETS})
