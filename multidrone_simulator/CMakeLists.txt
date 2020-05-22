cmake_minimum_required(VERSION 2.8.3)
project(multidrone_simulator)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
	cmake_modules
	gazebo_msgs
	geometry_msgs
	roscpp
	sensor_msgs
	trajectory_msgs
	eigen_conversions
	tf
  	uav_abstraction_layer
)

find_package(Eigen3 REQUIRED)
#find_package(gazebo REQUIRED)

# Check for mavros version for compatibility with older versions
execute_process(COMMAND rosversion mavros
		OUTPUT_VARIABLE mavros_version
		OUTPUT_STRIP_TRAILING_WHITESPACE
		)
if ("${mavros_version}" VERSION_LESS "0.25.0")
  add_definitions("-DMAVROS_VERSION_BELOW_0_25_0=\"${mavros_version}\"")
endif()

catkin_package(
    INCLUDE_DIRS #${Eigen3_INCLUDE_DIRS}
    CATKIN_DEPENDS gazebo_msgs geometry_msgs roscpp sensor_msgs trajectory_msgs eigen_conversions tf uav_abstraction_layer
)

# set(PROTOBUF_IMPORT_DIRS "")
# foreach(ITR ${GAZEBO_INCLUDE_DIRS})
#   if(ITR MATCHES ".*gazebo-[0-9.]+$")
#     set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
#   endif()
# endforeach()

# # PROTOBUF_IMPORT_DIRS has to be set before
# # find_package is called
# find_package(Protobuf REQUIRED)
# pkg_check_modules(PROTOBUF protobuf)

# if ("${PROTOBUF_VERSION}" VERSION_LESS "2.5.0")
#   message(FATAL_ERROR "protobuf version: ${PROTOBUF_VERSION} not compatible, must be >= 2.5.0")
# endif()

# if("${GAZEBO_VERSION}" VERSION_LESS "6.0")
#   message(FATAL_ERROR "You need at least Gazebo 6.0. Your version: ${GAZEBO_VERSION}")
# endif()

###########
## BUILD ##
###########

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS} -std=c++11 -Wno-deprecated-declarations")
# -Wno-deprecated-declarations: Shut up warnings about std::binder1st, std::binder2nd.

# set(GAZEBO_MSG_INCLUDE_DIRS)
# foreach(ITR ${GAZEBO_INCLUDE_DIRS})
#   if(ITR MATCHES ".*gazebo-[0-9.]+$")
#     set(GAZEBO_MSG_INCLUDE_DIRS "${ITR}/gazebo/msgs")
#   endif()
# endforeach()

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS})

# foreach(dir launch resource worlds)
#    install(DIRECTORY ${dir}/
#       DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
# endforeach(dir)

