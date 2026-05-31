cmake_minimum_required(VERSION 3.0.2)
project(fastnav_gz_bridge)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  nav_msgs
)

find_package(PkgConfig REQUIRED)

pkg_check_modules(GZ_TRANSPORT REQUIRED gz-transport13)
pkg_check_modules(GZ_MSGS REQUIRED gz-msgs10)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES fastnav_gz_bridge_core
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs nav_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GZ_TRANSPORT_INCLUDE_DIRS}
  ${GZ_MSGS_INCLUDE_DIRS}
)

link_directories(
  ${GZ_TRANSPORT_LIBRARY_DIRS}
  ${GZ_MSGS_LIBRARY_DIRS}
)

add_definitions(
  ${GZ_TRANSPORT_CFLAGS_OTHER}
  ${GZ_MSGS_CFLAGS_OTHER}
)

add_library(fastnav_gz_bridge_core
  src/gz_pointcloud_bridge.cpp
  src/gz_odom_bridge.cpp
)

target_link_libraries(fastnav_gz_bridge_core
  ${catkin_LIBRARIES}
  ${GZ_TRANSPORT_LIBRARIES}
  ${GZ_MSGS_LIBRARIES}
)

add_executable(gz_pointcloud_bridge_node
  src/gz_pointcloud_bridge_node.cpp
)

target_link_libraries(gz_pointcloud_bridge_node
  fastnav_gz_bridge_core
  ${catkin_LIBRARIES}
  ${GZ_TRANSPORT_LIBRARIES}
  ${GZ_MSGS_LIBRARIES}
)

add_executable(gz_odom_bridge_node
  src/gz_odom_bridge_node.cpp
)

target_link_libraries(gz_odom_bridge_node
  fastnav_gz_bridge_core
  ${catkin_LIBRARIES}
  ${GZ_TRANSPORT_LIBRARIES}
  ${GZ_MSGS_LIBRARIES}
)
