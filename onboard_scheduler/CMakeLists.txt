cmake_minimum_required(VERSION 2.8.3)
project(onboard_scheduler)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

add_definitions(-DUSING_IN_MULTIDRONE_PROJECT)

find_package(catkin REQUIRED COMPONENTS
  multidrone_msgs
  roscpp
  rospy
  tf
  roslaunch
  std_msgs
  std_srvs
  uav_abstraction_layer
  multidrone_planning
)

roslaunch_add_file_check(launch USE_TEST_DEPENDENCIES)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES scheduler_library
  CATKIN_DEPENDS multidrone_msgs roscpp rospy tf std_msgs std_srvs uav_abstraction_layer multidrone_planning
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(scheduler_library src/onboard_scheduler.cpp)
add_dependencies(scheduler_library ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(onboard_scheduler_node src/main.cpp)
target_link_libraries(onboard_scheduler_node scheduler_library ${catkin_LIBRARIES})
add_dependencies(onboard_scheduler_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
