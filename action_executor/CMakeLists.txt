cmake_minimum_required(VERSION 2.8.3)
project(action_executor)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  multidrone_msgs
  roscpp
  rospy
  roslaunch
  uav_abstraction_layer
  nav_msgs
  geometry_msgs
  tf2_eigen
  tf2_msgs
  tf2
  eigen_conversions
)

add_service_files(
	FILES
	TargetFeedbackMode.srv
)

# Check for mavros version for compatibility with older versions
execute_process(COMMAND rosversion mavros
		OUTPUT_VARIABLE mavros_version
		OUTPUT_STRIP_TRAILING_WHITESPACE
		)
if ("${mavros_version}" VERSION_LESS "0.25.0")
  add_definitions("-DMAVROS_VERSION_BELOW_0_25_0=\"${mavros_version}\"")
endif()

generate_messages(
	DEPENDENCIES
	geometry_msgs
)

catkin_package(
  LIBRARIES 
  CATKIN_DEPENDS multidrone_msgs tf2_eigen tf2_msgs roscpp rospy uav_abstraction_layer
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIR}
)

add_executable(action_executor_node src/action_executor_node.cpp src/action_executor.cpp src/trailer.cpp)
target_link_libraries(action_executor_node ${catkin_LIBRARIES} ${Eigen_LIBRARIES})
add_dependencies(action_executor_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

