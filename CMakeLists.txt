cmake_minimum_required(VERSION 2.8.3)
project(snap)

#add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  kobuki_msgs
  ar_track_alvar_msgs
  message_generation
  epic
)

catkin_python_setup()

add_message_files(
  FILES
  ActionType.msg
  ObservationActionComplete.msg
  ObservationDetectedObjects.msg
)

add_service_files(
  FILES
  ActionMove.srv
  ActionMoveInGrid.srv
  ActionNavigate.srv
  ActionNavigateToRegion.srv
  ActionPush.srv
  ActionAlign.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
  kobuki_msgs
  ar_track_alvar_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES snap
  CATKIN_DEPENDS message_runtime epic ar_track_alvar_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(
  PROGRAMS scripts/snap_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

