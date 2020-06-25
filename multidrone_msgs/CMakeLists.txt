cmake_minimum_required(VERSION 2.8.3)
project(multidrone_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  geographic_msgs
  geometry_msgs
  mavros_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
  actionlib_msgs
  actionlib
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  ActionStatus.msg
  AlarmType.msg
  CameraStatus.msg
  DroneAction.msg
  DroneActionArray.msg
  DroneActionStatus.msg
  DroneTelemetry.msg
  Event.msg
  FramingType.msg
  GimbalStatus.msg
  MissionError.msg
  MissionStatus.msg
  MultidronePlan.msg
  SASRole.msg
  ShootingAction.msg
  ShootingActionError.msg
  ShootingParameter.msg
  ShootingRole.msg
  ShootingType.msg
  SystemStatus.msg
  TargetGpsState.msg
  TargetIdentifierType.msg
  TargetState.msg
  TargetStateArray.msg
  TargetType.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  CameraControl.srv
  DirectorEvent.srv
  FollowTarget.srv
  GetSemanticMap.srv
  ManualControls.srv
  PostSemanticAnnotations.srv
  PushDroneAction.srv
  SafetyCheck.srv
  SelectRole.srv
  SendXML.srv
  SetFramingType.srv
  SupervisorAlarm.srv
  ValidateMission.srv
  Clear.srv
)

## Generate actions in the 'action' folder
 add_action_files(
   DIRECTORY
   action
   FILES
   Execute.action
 )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  diagnostic_msgs geographic_msgs geometry_msgs mavros_msgs nav_msgs sensor_msgs std_msgs actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS diagnostic_msgs geographic_msgs geometry_msgs mavros_msgs nav_msgs roscpp rospy sensor_msgs std_msgs std_srvs message_runtime
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
)
