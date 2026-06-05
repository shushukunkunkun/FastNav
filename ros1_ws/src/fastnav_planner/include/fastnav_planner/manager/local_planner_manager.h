/*
 * local_planner_manager.h
 *
 * 本文件声明 LocalPlannerManager 类。
 * LocalPlannerManager 是 PlannerFSM 下方的算法组织层，负责把状态机传入的 odom、cloud、goal
 * 组织成一次完整局部规划流程：
 * 1. 持有并更新 fastnav_mapping::VoxelMap，维护 planner 内部局部体素地图；
 * 2. 持有 AStarPlanner，直接通过 VoxelMap 查询 inflated occupancy，完成 3D A*；
 * 3. 持有 PathOptimizer，对 A* 原始路径做 shortcut、safe corridor 和 MINCO/GCOPTER 优化；
 * 4. 维护 global_data_ 和 local_data_，用 MINCO 保存全局参考轨迹和当前接受的局部轨迹；
 * 5. 缓存当前 odom、目标点、规划路径、搜索节点和 debug map；
 * 6. 向 FSM 提供 ROS 消息形式的 path / debug cloud，但本类自身不订阅也不发布 ROS topic。
 *
 * 架构上，FSM 负责“何时规划和发布”，manager 负责“如何更新地图和生成路径”。
 * 这种设计让高频碰撞查询保持在同一进程内存调用中，避免 planner 通过 ROS service 高频查询 mapping node。
 */

#pragma once

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <fastnav_mapping/voxel_map.h>
#include <traj_utils/plan_container.hpp>

#include "fastnav_planner/frontend/astar_planner.h"
#include "fastnav_planner/optimizer/path_optimizer.h"

namespace fastnav_planner
{

// LocalPlannerManager 负责组合 VoxelMap、AStarPlanner 和 PathOptimizer。
// ROS node / FSM 只把 odom、cloud、goal 转交给 manager，具体地图更新、A* 搜索和路径后处理都在同一进程内完成。
class LocalPlannerManager
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using Ptr = std::shared_ptr<LocalPlannerManager>;

    struct ReplanOptions
    {
        bool use_current_traj{false};
        bool use_random_init{false};
        int attempt{0};
        int continuous_failures{0};
    };

    // 初始化 manager：读取参数，创建 VoxelMap、AStarPlanner、PathOptimizer，并完成依赖注入。
    void init(ros::NodeHandle& nh, ros::NodeHandle& pnh);

    // 更新当前 odom；当前位姿会作为局部地图中心和规划起点。
    void updateOdom(const nav_msgs::OdometryConstPtr& msg);

    // 使用 filtered cloud 更新内部 VoxelMap，并刷新 debug occupied / inflated cloud。
    void updateCloud(const sensor_msgs::PointCloud2ConstPtr& msg);

    // 从当前 odom 位置规划到 goal；内部执行 A* 并调用 PathOptimizer。
    bool planToGoal(const Eigen::Vector3d& goal);

    // 带重规划选项的入口。use_random_init 会改变传给 corridor/MINCO 的参考路径，避免反复卡在同一局部解。
    bool planToGoal(const Eigen::Vector3d& goal, const ReplanOptions& options);

    // 将 current_path_ 打包为 nav_msgs::Path，供 FSM 发布。
    nav_msgs::Path getPathMsg() const;

    // 返回当前路径点列，供同进程内后续控制或轨迹模块直接使用。
    std::vector<Eigen::Vector3d> getPath() const;

    // 外部设置当前路径，作为后续轨迹优化 / 控制接口的预留入口。
    void setPath(const std::vector<Eigen::Vector3d>& path);

    // 返回 A* 搜索节点点云，用于 RViz 查看搜索范围。
    sensor_msgs::PointCloud2 getSearchedNodesCloud() const;

    // 返回 planner 内部地图的 occupied 体素点云。
    sensor_msgs::PointCloud2 getDebugOccupiedCloud() const;

    // 返回 planner 内部地图的 inflated 体素点云；A* 碰撞判断主要基于这类数据。
    sensor_msgs::PointCloud2 getDebugInflatedCloud() const;

    // 检查当前路径的每条线段是否仍然避开 inflated map，供 FSM 的 checkCollisionCallback() 调用。
    bool isCurrentPathCollisionFree(double step_size) const;

    // 状态查询接口，供 FSM 判断是否具备规划条件。
    bool hasOdom() const { return has_odom_; }
    bool hasMap() const { return has_map_; }
    bool hasPath() const { return has_path_; }

    // 是否在每次新点云到来后尝试重规划。
    bool replanOnCloud() const { return replan_on_cloud_; }

    // planner 使用的统一坐标系，当前为 odom。
    std::string frameId() const { return frame_id_; }

    // 返回最近一次失败原因，供 FSM 打印日志。
    std::string lastError() const { return last_error_; }

    // 从 current_odom_ 中提取当前位置，供 FSM 设置触发点、急停点和日志输出。
    Eigen::Vector3d currentPosition() const;

    // 暴露算法对象指针，便于后续轨迹模块或测试代码复用内部地图和优化器。
    std::shared_ptr<fastnav_mapping::VoxelMap> voxelMap() const { return voxel_map_; }
    std::shared_ptr<PathOptimizer> pathOptimizer() const { return path_optimizer_; }

    // 对齐 EGO-Planner 的数据组织方式：manager 负责维护全局参考轨迹和当前局部轨迹。
    // FastNav 当前版本使用 MINCO 表达轨迹，底层为分段五次多项式 $p_i(t)$。
    fastnav::GlobalTrajData global_data_;
    fastnav::LocalTrajData local_data_;

private:
    // 读取地图、A*、优化器参数。
    void loadParameters(ros::NodeHandle& nh, ros::NodeHandle& pnh);

    // 从 VoxelMap 导出当前 occupied / inflated 体素中心并缓存为 PointCloud2。
    void updateDebugClouds(const ros::Time& stamp);

    // 将 Eigen 点列转换为 PointCloud2，复用于 searched nodes 和 voxel debug cloud。
    sensor_msgs::PointCloud2 centersToCloud(const std::vector<Eigen::Vector3d>& centers,
                                            const ros::Time& stamp) const;

    // 根据 EGO-v2 的随机中点思想，对 A* 路径的中间控制点做小扰动，生成不同的 MINCO/corridor 初始化参考。
    std::vector<Eigen::Vector3d> buildOptimizationReferencePath(const std::vector<Eigen::Vector3d>& raw_path,
                                                                const ReplanOptions& options) const;

    // 将 PathOptimizer 输出的 MINCO 轨迹写入 local_data_；若 MINCO 不存在则只保留几何路径。
    void updateTrajInfo(const PathOptimizer::OptimizationResult& result,
                        const ros::Time& time_now);

    // 用本次优化结果更新全局参考数据；当前以局部 MINCO 作为任务参考，后续可替换为全局 MINCO。
    void updateGlobalTrajInfo(const PathOptimizer::OptimizationResult& result,
                              const ros::Time& time_now);

private:
    // 核心算法对象。manager 通过 shared_ptr 管理生命周期，并把 voxel_map_ 注入给 AStarPlanner / PathOptimizer。
    std::shared_ptr<fastnav_mapping::VoxelMap> voxel_map_;
    std::shared_ptr<AStarPlanner> astar_planner_;
    std::shared_ptr<PathOptimizer> path_optimizer_;

    // 坐标系和重规划策略。
    std::string frame_id_{"odom"};
    bool replan_on_cloud_{false};

    // VoxelMap 几何参数和占据更新参数。
    double resolution_{0.2};
    double local_x_size_{20.0};
    double local_y_size_{20.0};
    double local_z_size_{6.0};
    double local_z_min_{-2.0};
    double local_z_max_{4.0};
    double drone_radius_{0.35};
    double safety_margin_{0.15};
    double map_p_hit_{0.70};
    double map_p_miss_{0.45};
    double map_p_min_{0.12};
    double map_p_max_{0.97};
    double map_p_occ_{0.65};
    double map_temporal_decay_log_{0.05};

    // A* 和路径优化配置，会在 init() 中传给对应算法对象。
    AStarPlanner::Config astar_config_;
    PathOptimizer::Config optimizer_config_;

    // MINCO 采样参数。current_path_ 用于 RViz 和碰撞检查，优先保存 MINCO 采样点。
    double minco_sample_dt_{0.05};
    double random_init_scale_{0.8};

    // manager 内部数据有效性标志。
    bool has_odom_{false};
    bool has_map_{false};
    bool has_path_{false};

    // 当前状态缓存：里程计、最近点云时间、最近目标点。
    nav_msgs::Odometry current_odom_;
    ros::Time last_cloud_stamp_;
    Eigen::Vector3d last_goal_{0.0, 0.0, 0.0};

    // 规划结果和可视化缓存。
    std::vector<Eigen::Vector3d> current_path_;
    PathOptimizer::OptimizationResult last_optimization_result_;
    sensor_msgs::PointCloud2 debug_occupied_cloud_;
    sensor_msgs::PointCloud2 debug_inflated_cloud_;
    sensor_msgs::PointCloud2 searched_nodes_cloud_;

    // 最近一次规划或地图更新失败原因。
    std::string last_error_;
};

}  // namespace fastnav_planner
