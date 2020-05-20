cmake_minimum_required(VERSION 2.8.3)
project(dashboard_interface)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  multidrone_msgs
  geometry_msgs
  roscpp
  rospy
  tf
  roslaunch
)

roslaunch_add_file_check(launch USE_TEST_DEPENDENCIES)

catkin_package(
  CATKIN_DEPENDS multidrone_msgs sensor_msgs geometry_msgs roscpp rospy tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(dummy_mc_node src/dummy_mc_node)
target_link_libraries(dummy_mc_node ${catkin_LIBRARIES})
add_dependencies(dummy_mc_node ${catkin_EXPORTED_TARGETS})