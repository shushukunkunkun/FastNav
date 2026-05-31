cmake_minimum_required(VERSION 3.0.2)
project(fastnav_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  message_generation
)

add_message_files(
  FILES
  ControlCommand.msg
  ObstacleInfo.msg
  PlannerState.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
)

catkin_package(
  CATKIN_DEPENDS
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  message_runtime
)