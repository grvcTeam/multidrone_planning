cmake_minimum_required(VERSION 2.8.3)
project(multidrone_planning)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

add_definitions(-DUSING_IN_MULTIDRONE_PROJECT)

find_package(catkin REQUIRED COMPONENTS
  multidrone_msgs
  roscpp
  rospy
  tf
  roslaunch
  multidrone_kml_parser
  roslib
)

find_package(PythonLibs 2.7)

roslaunch_add_file_check(launch USE_TEST_DEPENDENCIES)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES path_planner
  CATKIN_DEPENDS multidrone_msgs roscpp rospy tf multidrone_kml_parser roslib
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(mission_controller src/mission_controller_node.cpp src/mission_controller.cpp src/high_level_planner.cpp)
target_include_directories(mission_controller PRIVATE ${PYTHON_INCLUDE_DIRS})
target_link_libraries(mission_controller ${catkin_LIBRARIES} ${PYTHON_LIBRARIES} path_planner)
add_dependencies(mission_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(path_planner src/path_planner.cpp)
target_include_directories(path_planner PRIVATE ${PYTHON_INCLUDE_DIRS})
target_link_libraries(path_planner ${catkin_LIBRARIES} ${PYTHON_LIBRARIES})
add_dependencies(path_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
