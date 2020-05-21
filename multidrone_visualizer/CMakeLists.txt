cmake_minimum_required(VERSION 2.8.3)
project(multidrone_visualizer)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  geometry_msgs
  std_msgs
  roscpp
  visualization_msgs
  tf
  multidrone_msgs
  tf2_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES visualizer_library
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}

)

add_library(visualizer_library src/visualizer.cpp)
target_include_directories(visualizer_library PRIVATE ${PYTHON_INCLUDE_DIRS})
add_dependencies(visualizer_library ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(visualizer_node src/multidrone_visualizer_node.cpp)
target_link_libraries(visualizer_node ${catkin_LIBRARIES} visualizer_library)
add_dependencies(visualizer_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})