cmake_minimum_required(VERSION 2.8.3)
project(gimbal_camera_interface)

# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  interface_test
	multidrone_msgs
  roscpp
  std_msgs
	cmake_modules
  sensor_msgs
  eigen_conversions
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

catkin_package(
	LIBRARIES interface_test
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs eigen_conversions
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)

add_executable(gimbal_interface_node src/gimbal_camera_interface_node.cpp src/gimbal_camera_interface.cpp)
add_executable(gimbal_setup src/gimbal_setup.cpp)
add_executable(gimbal_calibration src/gimbal_calibration.cpp)

add_dependencies(gimbal_interface_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(gimbal_interface_node
 ${catkin_LIBRARIES}
 ${Eigen3_LIBRARIES}
 ${Boost_LIBRARIES}
 usbp-1
)

target_link_libraries(gimbal_calibration
 ${catkin_LIBRARIES}
 ${Eigen3_LIBRARIES}
 ${Boost_LIBRARIES}
 usbp-1
)

target_link_libraries(gimbal_setup
 ${catkin_LIBRARIES}
 ${Eigen3_LIBRARIES}
 ${Boost_LIBRARIES}
)
