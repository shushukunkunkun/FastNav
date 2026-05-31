cmake_minimum_required(VERSION 3.0.2)
project(fastnav_planner)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  fastnav_mapping
  geometry_msgs
  nav_msgs
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
)

find_package(PCL REQUIRED COMPONENTS common)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES fastnav_planner_core
  CATKIN_DEPENDS fastnav_mapping geometry_msgs nav_msgs pcl_conversions pcl_ros roscpp sensor_msgs std_msgs
  DEPENDS EIGEN3 PCL
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

add_library(fastnav_planner_core
  src/astar_planner.cpp
  src/local_planner_manager.cpp
  src/path_optimizer.cpp
  src/planner_fsm.cpp
)

add_dependencies(fastnav_planner_core
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(fastnav_planner_core
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(fastnav_local_planner_node
  src/local_planner_node.cpp
)

add_dependencies(fastnav_local_planner_node
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(fastnav_local_planner_node
  fastnav_planner_core
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
