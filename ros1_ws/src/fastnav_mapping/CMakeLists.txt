cmake_minimum_required(VERSION 3.0.2)
project(fastnav_mapping)

# 当前系统的 ROS/log4cxx 头文件需要 C++17；mapping 模块本身只做局部体素地图，不依赖仿真专有接口。
add_compile_options(-std=c++17)

# fastnav_mapping 只消费 FastNav 标准接口：$/fastnav/perception/cloud_filtered$ 和 $/fastnav/state/odom$。
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  visualization_msgs
)

# PCL 用于把体素中心集合 $S = {p_i}$ 发布为 PointCloud2，可视化 occupied 和 inflated 区域。
find_package(PCL REQUIRED COMPONENTS common)
# Eigen3 用于体素索引 $i$ 和 odom 坐标 $p$ 的向量运算。
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES voxel_map
  CATKIN_DEPENDS geometry_msgs nav_msgs pcl_conversions pcl_ros roscpp sensor_msgs visualization_msgs
  DEPENDS EIGEN3 PCL
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

# VoxelMap 是纯地图数据结构，核心关系是 $i = floor((p - origin) / resolution)$。
add_library(voxel_map
  src/voxel_map.cpp
)

target_link_libraries(voxel_map
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

# local_voxel_map_node 负责把 cloud_filtered 写入 VoxelMap，并发布 occupied_cloud / inflated_cloud。
add_executable(local_voxel_map_node
  src/local_voxel_map_node.cpp
)

target_link_libraries(local_voxel_map_node
  voxel_map
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
