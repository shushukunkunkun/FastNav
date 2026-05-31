cmake_minimum_required(VERSION 3.0.2)
project(fastnav_perception)

# 当前系统的 ROS/log4cxx 头文件会用到 $std::shared_mutex$，因此这里使用 C++17。
# 本包只做第一版感知预处理，不生成自定义消息，核心链路是 $P_mid360 -> P_odom -> P_filtered$。
add_compile_options(-std=c++17)

# catkin 依赖说明：$sensor_msgs$ 提供 PointCloud2，$tf2_ros$ 负责查询 TF，$tf2_sensor_msgs$ 负责按 $p_o = R p_l + t$ 转换点云。
# $pcl_conversions$ 和 $pcl_ros$ 用于 ROS 点云与 PCL 点云之间的转换。
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
)

# PCL 的 common 提供点云基础结构，filters 提供 NaN 去除、VoxelGrid 等滤波器。
# 这里的降采样可以理解为把空间划分成边长为 $l$ 的体素，并用每个体素中的代表点近似原始点集。
find_package(PCL REQUIRED COMPONENTS common filters)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    nav_msgs
    pcl_conversions
    pcl_ros
    roscpp
    sensor_msgs
    tf2
    tf2_ros
    tf2_sensor_msgs
  DEPENDS PCL
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

# cloud_transform_node 完成坐标变换：输入点 $p_l$ 在 mid360_link 下，输出点 $p_o$ 在 odom 下。
add_executable(cloud_transform_node
  src/cloud_transform_node.cpp
)

target_link_libraries(cloud_transform_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

# cloud_filter_node 完成预处理：保留满足 $r_min <= ||p - s|| <= r_max$ 和局部裁剪盒约束的点，再做体素降采样。
add_executable(cloud_filter_node
  src/cloud_filter_node.cpp
)

target_link_libraries(cloud_filter_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
