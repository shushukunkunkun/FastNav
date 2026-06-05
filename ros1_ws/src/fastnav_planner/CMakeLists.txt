cmake_minimum_required(VERSION 3.0.2)
project(fastnav_planner)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  fastnav_mapping
  fastnav_msgs
  geometry_msgs
  nav_msgs
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  traj_utils
)

find_package(PCL REQUIRED COMPONENTS common)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES fastnav_planner_core
  CATKIN_DEPENDS fastnav_mapping fastnav_msgs geometry_msgs nav_msgs pcl_conversions pcl_ros roscpp sensor_msgs std_msgs traj_utils
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
  src/frontend/astar_planner.cpp
  src/manager/local_planner_manager.cpp
  src/optimizer/path_optimizer.cpp
  src/optimizer/trajectory_feasibility_checker.cpp
  src/fsm/planner_fsm.cpp
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
