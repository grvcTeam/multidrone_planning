cmake_minimum_required(VERSION 2.8.3)
project(multidrone_simulator)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS gazebo_plugins std_msgs
  roscpp 
  gazebo_ros
  multidrone_msgs
)
include_directories(include ${catkin_INCLUDE_DIRS})

## Find gazebo
if(POLICY CMP0054)
  cmake_policy(SET CMP0054 NEW)
endif()
find_package(gazebo REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

catkin_package(
  DEPENDS 
    roscpp 
    gazebo_ros 
)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/target_plugin.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})


add_executable(target_node src/target_node.cpp)

target_link_libraries(target_node
${catkin_LIBRARIES} ${EXTRALIB_BIN} ${PYTHON_LIBRARIES} ${Eigen3_LIBRARIES} ${GAZEBO_LIBRARIES} )

add_dependencies(target_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
