cmake_minimum_required(VERSION 3.0.2)
project(fastnav_tf)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  tf2_ros
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp tf2_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(fastnav_tf_broadcaster_node
  src/fastnav_tf_broadcaster_node.cpp
)

target_link_libraries(fastnav_tf_broadcaster_node
  ${catkin_LIBRARIES}
)
