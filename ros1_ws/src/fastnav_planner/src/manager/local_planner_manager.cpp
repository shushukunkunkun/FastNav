/*
 * local_planner_manager.cpp
 *
 * 本文件实现 LocalPlannerManager 类。
 * LocalPlannerManager 是 PlannerFSM 下方的算法组织层，不直接订阅 ROS topic，也不发布 ROS topic。
 * 它的工作是把“状态机事件”转换为“算法对象之间的内存调用”：
 * 1. 持有 fastnav_mapping::VoxelMap，用 filtered cloud 更新 planner 内部局部体素地图；
 * 2. 持有 AStarPlanner，直接查询 VoxelMap 的 inflated occupancy 做 3D A* 搜索；
 * 3. 持有 PathOptimizer，对 A* 原始路径做 shortcut、safe corridor 和 MINCO/GCOPTER 优化；
 * 4. 维护 global_data_ 和 local_data_，用 MINCO 保存全局参考轨迹和当前接受的局部轨迹；
 * 5. 缓存 current_odom_、last_goal_、current_path_、debug cloud 和 searched nodes；
 * 6. 向 PlannerFSM 提供 getPathMsg()、getDebugInflatedCloud() 等结果接口。
 *
 * 架构上，FSM 负责“什么时候规划、什么时候重规划”，manager 负责“如何更新地图和生成路径”。
 * 这样 planner 的高频碰撞查询通过共享指针在同一进程内完成，不需要 ROS service / topic 往返。
 */

#include "fastnav_planner/manager/local_planner_manager.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace fastnav_planner
{

// 初始化 manager 内部算法对象：读取参数，创建 VoxelMap、AStarPlanner 和 PathOptimizer，并完成指针绑定。
void LocalPlannerManager::init(ros::NodeHandle& nh, ros::NodeHandle& pnh)
{
    loadParameters(nh, pnh);

    voxel_map_ = std::make_shared<fastnav_mapping::VoxelMap>();
    voxel_map_->init(resolution_,
                     local_x_size_,
                     local_y_size_,
                     local_z_size_,
                     local_z_min_,
                     local_z_max_,
                     drone_radius_,
                     safety_margin_);
    // planner 内部 VoxelMap 不再每帧完全重建，而是像 EGO-Planner 的 GridMap 一样长期持有局部缓存。
    // 点云命中用 log-odds 累计 $l_i <- clamp(l_i + log(p_hit/(1-p_hit)), l_min, l_max)$；
    // 当前版本暂不做 raycasting free-space，因此用 temporal_decay_log 把旧障碍缓慢拉回未知 $l=0$。
    voxel_map_->setOccupancyUpdateParams(map_p_hit_,
                                         map_p_miss_,
                                         map_p_min_,
                                         map_p_max_,
                                         map_p_occ_,
                                         map_temporal_decay_log_);

    astar_planner_ = std::make_shared<AStarPlanner>();
    astar_planner_->setMap(voxel_map_);
    astar_planner_->setConfig(astar_config_);

    path_optimizer_ = std::make_shared<PathOptimizer>();
    path_optimizer_->setConfig(optimizer_config_);

    global_data_.reset();
    local_data_.reset();

    ROS_INFO("[FastNav][LocalPlannerManager] Initialized. resolution=%.3f, frame=%s",
             resolution_, frame_id_.c_str());
}

// 读取局部地图、A*、优化器参数；全局 yaml 提供默认值，私有参数提供节点级覆盖入口。
void LocalPlannerManager::loadParameters(ros::NodeHandle& nh, ros::NodeHandle& pnh)
{
    nh.param<std::string>("/local_planner/frame_id", frame_id_, frame_id_);
    nh.param<bool>("/local_planner/replan_on_cloud", replan_on_cloud_, replan_on_cloud_);

    nh.param<double>("/local_planner/map/resolution", resolution_, resolution_);
    nh.param<double>("/local_planner/map/local_x_size", local_x_size_, local_x_size_);
    nh.param<double>("/local_planner/map/local_y_size", local_y_size_, local_y_size_);
    nh.param<double>("/local_planner/map/local_z_size", local_z_size_, local_z_size_);
    nh.param<double>("/local_planner/map/local_z_min", local_z_min_, local_z_min_);
    nh.param<double>("/local_planner/map/local_z_max", local_z_max_, local_z_max_);
    nh.param<double>("/local_planner/map/drone_radius", drone_radius_, drone_radius_);
    nh.param<double>("/local_planner/map/safety_margin", safety_margin_, safety_margin_);
    nh.param<double>("/local_planner/map/p_hit", map_p_hit_, map_p_hit_);
    nh.param<double>("/local_planner/map/p_miss", map_p_miss_, map_p_miss_);
    nh.param<double>("/local_planner/map/p_min", map_p_min_, map_p_min_);
    nh.param<double>("/local_planner/map/p_max", map_p_max_, map_p_max_);
    nh.param<double>("/local_planner/map/p_occ", map_p_occ_, map_p_occ_);
    nh.param<double>("/local_planner/map/temporal_decay_log", map_temporal_decay_log_, map_temporal_decay_log_);

    nh.param<double>("/planner/local_map/resolution", resolution_, resolution_);
    nh.param<double>("/planner/local_map/local_x_size", local_x_size_, local_x_size_);
    nh.param<double>("/planner/local_map/local_y_size", local_y_size_, local_y_size_);
    nh.param<double>("/planner/local_map/local_z_size", local_z_size_, local_z_size_);
    nh.param<double>("/planner/local_map/local_z_min", local_z_min_, local_z_min_);
    nh.param<double>("/planner/local_map/local_z_max", local_z_max_, local_z_max_);

    nh.param<bool>("/local_planner/astar/allow_diagonal", astar_config_.allow_diagonal, astar_config_.allow_diagonal);
    nh.param<bool>("/local_planner/astar/check_line_collision", astar_config_.check_line_collision, astar_config_.check_line_collision);
    nh.param<double>("/local_planner/astar/heuristic_weight", astar_config_.heuristic_weight, astar_config_.heuristic_weight);
    nh.param<double>("/local_planner/astar/min_clearance", astar_config_.min_clearance, astar_config_.min_clearance);
    nh.param<double>("/local_planner/astar/line_check_step", astar_config_.line_check_step, astar_config_.line_check_step);
    nh.param<int>("/local_planner/astar/max_search_nodes", astar_config_.max_search_nodes, astar_config_.max_search_nodes);
    nh.param<double>("/planner/astar/heuristic_weight", astar_config_.heuristic_weight, astar_config_.heuristic_weight);
    nh.param<double>("/planner/astar/min_clearance", astar_config_.min_clearance, astar_config_.min_clearance);
    nh.param<int>("/planner/astar/max_search_nodes", astar_config_.max_search_nodes, astar_config_.max_search_nodes);
    nh.param<bool>("/local_planner/optimizer/enable", optimizer_config_.enable, optimizer_config_.enable);
    nh.param<bool>("/local_planner/optimizer/shortcut", optimizer_config_.shortcut, optimizer_config_.shortcut);
    nh.param<double>("/local_planner/optimizer/line_check_step", optimizer_config_.line_check_step, optimizer_config_.line_check_step);
    nh.param<bool>("/local_planner/optimizer/minco_enable", optimizer_config_.minco_enable, optimizer_config_.minco_enable);
    nh.param<bool>("/local_planner/optimizer/require_minco", optimizer_config_.require_minco, optimizer_config_.require_minco);
    nh.param<double>("/local_planner/optimizer/minco_sample_dt", optimizer_config_.minco_sample_dt, optimizer_config_.minco_sample_dt);
    nh.param<int>("/local_planner/optimizer/max_retry", optimizer_config_.max_retry, optimizer_config_.max_retry);
    nh.param<double>("/local_planner/optimizer/random_init_scale", random_init_scale_, random_init_scale_);
    nh.param<double>("/local_planner/optimizer/retry/corridor_range_scale", optimizer_config_.corridor_range_retry_scale, optimizer_config_.corridor_range_retry_scale);
    nh.param<double>("/local_planner/optimizer/retry/corridor_progress_scale", optimizer_config_.corridor_progress_retry_scale, optimizer_config_.corridor_progress_retry_scale);
    nh.param<double>("/local_planner/optimizer/retry/weight_time_scale", optimizer_config_.weight_time_retry_scale, optimizer_config_.weight_time_retry_scale);
    nh.param<double>("/local_planner/optimizer/retry/penalty_pos_scale", optimizer_config_.penalty_pos_retry_scale, optimizer_config_.penalty_pos_retry_scale);
    nh.param<double>("/local_planner/optimizer/retry/penalty_vel_scale", optimizer_config_.penalty_vel_retry_scale, optimizer_config_.penalty_vel_retry_scale);
    nh.param<double>("/local_planner/optimizer/retry/penalty_body_rate_scale", optimizer_config_.penalty_body_rate_retry_scale, optimizer_config_.penalty_body_rate_retry_scale);
    nh.param<double>("/local_planner/optimizer/retry/penalty_tilt_scale", optimizer_config_.penalty_tilt_retry_scale, optimizer_config_.penalty_tilt_retry_scale);
    nh.param<double>("/local_planner/optimizer/retry/penalty_thrust_scale", optimizer_config_.penalty_thrust_retry_scale, optimizer_config_.penalty_thrust_retry_scale);
    nh.param<double>("/local_planner/optimizer/corridor/progress", optimizer_config_.corridor.progress, optimizer_config_.corridor.progress);
    nh.param<double>("/local_planner/optimizer/corridor/range", optimizer_config_.corridor.range, optimizer_config_.corridor.range);
    nh.param<double>("/local_planner/optimizer/corridor/eps", optimizer_config_.corridor.eps, optimizer_config_.corridor.eps);
    nh.param<bool>("/local_planner/optimizer/corridor/enable_shortcut", optimizer_config_.corridor.enable_shortcut, optimizer_config_.corridor.enable_shortcut);
    nh.param<double>("/local_planner/optimizer/minco/weight_time", optimizer_config_.minco.weight_time, optimizer_config_.minco.weight_time);
    nh.param<double>("/local_planner/optimizer/minco/smoothing_eps", optimizer_config_.minco.smoothing_eps, optimizer_config_.minco.smoothing_eps);
    nh.param<int>("/local_planner/optimizer/minco/integral_intervals", optimizer_config_.minco.integral_intervals, optimizer_config_.minco.integral_intervals);
    nh.param<double>("/local_planner/optimizer/minco/rel_cost_tol", optimizer_config_.minco.rel_cost_tol, optimizer_config_.minco.rel_cost_tol);
    nh.param<double>("/local_planner/optimizer/minco/length_per_piece", optimizer_config_.minco.length_per_piece, optimizer_config_.minco.length_per_piece);
    nh.param<double>("/local_planner/optimizer/minco/max_vel", optimizer_config_.minco.max_vel, optimizer_config_.minco.max_vel);
    nh.param<double>("/local_planner/optimizer/minco/max_body_rate", optimizer_config_.minco.max_body_rate, optimizer_config_.minco.max_body_rate);
    nh.param<double>("/local_planner/optimizer/minco/max_tilt_angle", optimizer_config_.minco.max_tilt_angle, optimizer_config_.minco.max_tilt_angle);
    nh.param<double>("/local_planner/optimizer/minco/min_thrust", optimizer_config_.minco.min_thrust, optimizer_config_.minco.min_thrust);
    nh.param<double>("/local_planner/optimizer/minco/max_thrust", optimizer_config_.minco.max_thrust, optimizer_config_.minco.max_thrust);
    nh.param<double>("/local_planner/optimizer/minco/mass", optimizer_config_.minco.mass, optimizer_config_.minco.mass);
    nh.param<double>("/local_planner/optimizer/minco/gravity", optimizer_config_.minco.gravity, optimizer_config_.minco.gravity);
    nh.param<double>("/local_planner/optimizer/minco/horiz_drag", optimizer_config_.minco.horiz_drag, optimizer_config_.minco.horiz_drag);
    nh.param<double>("/local_planner/optimizer/minco/vert_drag", optimizer_config_.minco.vert_drag, optimizer_config_.minco.vert_drag);
    nh.param<double>("/local_planner/optimizer/minco/paras_drag", optimizer_config_.minco.paras_drag, optimizer_config_.minco.paras_drag);
    nh.param<double>("/local_planner/optimizer/minco/speed_eps", optimizer_config_.minco.speed_eps, optimizer_config_.minco.speed_eps);
    nh.param<double>("/local_planner/optimizer/minco/penalty_pos", optimizer_config_.minco.penalty_pos, optimizer_config_.minco.penalty_pos);
    nh.param<double>("/local_planner/optimizer/minco/penalty_vel", optimizer_config_.minco.penalty_vel, optimizer_config_.minco.penalty_vel);
    nh.param<double>("/local_planner/optimizer/minco/penalty_body_rate", optimizer_config_.minco.penalty_body_rate, optimizer_config_.minco.penalty_body_rate);
    nh.param<double>("/local_planner/optimizer/minco/penalty_tilt", optimizer_config_.minco.penalty_tilt, optimizer_config_.minco.penalty_tilt);
    nh.param<double>("/local_planner/optimizer/minco/penalty_thrust", optimizer_config_.minco.penalty_thrust, optimizer_config_.minco.penalty_thrust);
    nh.param<bool>("/local_planner/optimizer/feasibility/check_collision", optimizer_config_.feasibility.check_collision, optimizer_config_.feasibility.check_collision);
    nh.param<bool>("/local_planner/optimizer/feasibility/check_dynamics", optimizer_config_.feasibility.check_dynamics, optimizer_config_.feasibility.check_dynamics);
    nh.param<double>("/local_planner/optimizer/feasibility/sample_dt", optimizer_config_.feasibility.sample_dt, optimizer_config_.feasibility.sample_dt);
    nh.param<double>("/local_planner/optimizer/feasibility/collision_check_step", optimizer_config_.feasibility.collision_check_step, optimizer_config_.feasibility.collision_check_step);
    nh.param<double>("/local_planner/optimizer/feasibility/check_horizon_ratio", optimizer_config_.feasibility.check_horizon_ratio, optimizer_config_.feasibility.check_horizon_ratio);
    nh.param<double>("/local_planner/optimizer/feasibility/max_vel", optimizer_config_.feasibility.max_vel, optimizer_config_.feasibility.max_vel);
    nh.param<double>("/local_planner/optimizer/feasibility/max_acc", optimizer_config_.feasibility.max_acc, optimizer_config_.feasibility.max_acc);
    nh.param<double>("/local_planner/optimizer/feasibility/max_jerk", optimizer_config_.feasibility.max_jerk, optimizer_config_.feasibility.max_jerk);
    nh.param<double>("/local_planner/optimizer/feasibility/tolerance", optimizer_config_.feasibility.tolerance, optimizer_config_.feasibility.tolerance);

    nh.param<double>("/planner/safe_corridor/progress", optimizer_config_.corridor.progress, optimizer_config_.corridor.progress);
    nh.param<double>("/planner/safe_corridor/range", optimizer_config_.corridor.range, optimizer_config_.corridor.range);
    nh.param<int>("/planner/replan/max_retry", optimizer_config_.max_retry, optimizer_config_.max_retry);
    nh.param<double>("/planner/replan/random_init_scale", random_init_scale_, random_init_scale_);
    nh.param<double>("/planner/replan/corridor_range_scale", optimizer_config_.corridor_range_retry_scale, optimizer_config_.corridor_range_retry_scale);
    nh.param<double>("/planner/replan/corridor_progress_scale", optimizer_config_.corridor_progress_retry_scale, optimizer_config_.corridor_progress_retry_scale);
    nh.param<double>("/planner/replan/weight_time_scale", optimizer_config_.weight_time_retry_scale, optimizer_config_.weight_time_retry_scale);
    nh.param<double>("/planner/replan/penalty_pos_scale", optimizer_config_.penalty_pos_retry_scale, optimizer_config_.penalty_pos_retry_scale);
    nh.param<double>("/planner/replan/penalty_vel_scale", optimizer_config_.penalty_vel_retry_scale, optimizer_config_.penalty_vel_retry_scale);
    nh.param<double>("/planner/minco/max_body_rate", optimizer_config_.minco.max_body_rate, optimizer_config_.minco.max_body_rate);
    nh.param<double>("/planner/minco/max_tilt_angle", optimizer_config_.minco.max_tilt_angle, optimizer_config_.minco.max_tilt_angle);
    nh.param<double>("/planner/minco/min_thrust", optimizer_config_.minco.min_thrust, optimizer_config_.minco.min_thrust);
    nh.param<double>("/planner/minco/max_thrust", optimizer_config_.minco.max_thrust, optimizer_config_.minco.max_thrust);
    nh.param<double>("/planner/minco/mass", optimizer_config_.minco.mass, optimizer_config_.minco.mass);
    nh.param<double>("/planner/physical_limits/max_vel", optimizer_config_.minco.max_vel, optimizer_config_.minco.max_vel);
    nh.param<double>("/planner/physical_limits/max_vel", optimizer_config_.feasibility.max_vel, optimizer_config_.feasibility.max_vel);
    nh.param<double>("/planner/physical_limits/max_acc", optimizer_config_.feasibility.max_acc, optimizer_config_.feasibility.max_acc);
    nh.param<double>("/planner/physical_limits/max_jerk", optimizer_config_.feasibility.max_jerk, optimizer_config_.feasibility.max_jerk);
    nh.param<double>("/planner/physical_limits/feasibility_tolerance", optimizer_config_.feasibility.tolerance, optimizer_config_.feasibility.tolerance);
    nh.param<double>("/planner/feasibility/sample_dt", optimizer_config_.feasibility.sample_dt, optimizer_config_.feasibility.sample_dt);
    nh.param<double>("/planner/feasibility/collision_check_step", optimizer_config_.feasibility.collision_check_step, optimizer_config_.feasibility.collision_check_step);
    nh.param<double>("/planner/feasibility/check_horizon_ratio", optimizer_config_.feasibility.check_horizon_ratio, optimizer_config_.feasibility.check_horizon_ratio);

    pnh.param<std::string>("frame_id", frame_id_, frame_id_);
    pnh.param<bool>("replan_on_cloud", replan_on_cloud_, replan_on_cloud_);
    pnh.param<double>("astar/min_clearance", astar_config_.min_clearance, astar_config_.min_clearance);
    pnh.param<bool>("optimizer/enable", optimizer_config_.enable, optimizer_config_.enable);
    pnh.param<bool>("optimizer/shortcut", optimizer_config_.shortcut, optimizer_config_.shortcut);
    pnh.param<double>("optimizer/line_check_step", optimizer_config_.line_check_step, optimizer_config_.line_check_step);
    pnh.param<bool>("optimizer/minco_enable", optimizer_config_.minco_enable, optimizer_config_.minco_enable);
    pnh.param<bool>("optimizer/require_minco", optimizer_config_.require_minco, optimizer_config_.require_minco);
    pnh.param<double>("optimizer/minco_sample_dt", optimizer_config_.minco_sample_dt, optimizer_config_.minco_sample_dt);
    pnh.param<int>("optimizer/max_retry", optimizer_config_.max_retry, optimizer_config_.max_retry);
    pnh.param<double>("optimizer/random_init_scale", random_init_scale_, random_init_scale_);

    minco_sample_dt_ = std::max(1.0e-3, optimizer_config_.minco_sample_dt);
    random_init_scale_ = std::max(0.0, random_init_scale_);
    astar_config_.min_clearance = std::max(0.0, astar_config_.min_clearance);
}

// 更新当前无人机状态；current_odom_ 后续用于地图中心 $c$ 和 A* 搜索起点。
void LocalPlannerManager::updateOdom(const nav_msgs::OdometryConstPtr& msg)
{
    current_odom_ = *msg;
    has_odom_ = true;
}

// 用最新 filtered cloud 更新 planner 内部 VoxelMap，并刷新 occupied / inflated debug cloud。
void LocalPlannerManager::updateCloud(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    if (!has_odom_)
    {
        last_error_ = "Waiting for odom before updating planner map.";
        return;
    }

    if (msg->header.frame_id != frame_id_)
    {
        last_error_ = "Cloud frame is not planner frame.";
        ROS_WARN_THROTTLE(1.0,
                          "[FastNav][LocalPlannerManager] Expected cloud frame %s, got %s.",
                          frame_id_.c_str(), msg->header.frame_id.c_str());
        return;
    }

    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);

    // 局部地图中心跟随无人机位置 $c$ 滑动，VoxelMap 会按整格偏移搬运旧缓存。
    // 因此这里不再 reset；旧障碍若持续被看见会继续累积，没被看见则通过 $l_i -> 0$ 的衰减逐渐遗忘。
    voxel_map_->setMapCenter(currentPosition());
    voxel_map_->decayOccupancy();

    for (const pcl::PointXYZ& point : cloud.points)
    {
        voxel_map_->setOccupied(Eigen::Vector3d(point.x, point.y, point.z));
    }
    voxel_map_->inflateObstacles();

    last_cloud_stamp_ = msg->header.stamp;
    has_map_ = true;
    updateDebugClouds(msg->header.stamp);
}

// 从当前无人机位置规划到目标点：先运行 A*，再调用 PathOptimizer 生成 safe corridor 和 MINCO 轨迹。
bool LocalPlannerManager::planToGoal(const Eigen::Vector3d& goal)
{
    ReplanOptions options;
    return planToGoal(goal, options);
}

bool LocalPlannerManager::planGlobalTraj(const Eigen::Vector3d& start_pos,
                                         const Eigen::Vector3d& start_vel,
                                         const Eigen::Vector3d& start_acc,
                                         const Eigen::Vector3d& end_pos,
                                         const Eigen::Vector3d& end_vel,
                                         const Eigen::Vector3d& end_acc)
{
    const Eigen::Vector3d delta = end_pos - start_pos;
    const double distance = delta.norm();
    if (distance < 1.0e-4)
    {
        last_error_ = "Global trajectory start and goal are too close.";
        return false;
    }

    const double max_vel = std::max(0.1, optimizer_config_.feasibility.max_vel);
    const double max_acc = std::max(0.1, optimizer_config_.feasibility.max_acc);
    // 单段五次 smooth reference 的峰值速度和加速度会高于 $d/T$ 与 $d/T^2$，
    // 因此这里按 $T=max(2d/v_{max}, sqrt(6d/a_{max}))$ 给全局参考留足时间裕度。
    const double duration = std::max(1.0,
                                     std::max(2.0 * distance / max_vel,
                                              std::sqrt(6.0 * distance / max_acc)));

    Eigen::Matrix3d A;
    A << std::pow(duration, 3), std::pow(duration, 4), std::pow(duration, 5),
         3.0 * std::pow(duration, 2), 4.0 * std::pow(duration, 3), 5.0 * std::pow(duration, 4),
         6.0 * duration, 12.0 * std::pow(duration, 2), 20.0 * std::pow(duration, 3);

    Piece<5>::CoefficientMat coeff;
    coeff.setZero();
    for (int axis = 0; axis < 3; ++axis)
    {
        const double a0 = start_pos(axis);
        const double a1 = start_vel(axis);
        const double a2 = 0.5 * start_acc(axis);

        Eigen::Vector3d b;
        b(0) = end_pos(axis) - (a0 + a1 * duration + a2 * duration * duration);
        b(1) = end_vel(axis) - (a1 + 2.0 * a2 * duration);
        b(2) = end_acc(axis) - (2.0 * a2);

        const Eigen::Vector3d high = A.lu().solve(b);
        // GCOPTER Piece<5> 的系数列顺序是 $[t^5,t^4,t^3,t^2,t,1]$。
        coeff(axis, 5) = a0;
        coeff(axis, 4) = a1;
        coeff(axis, 3) = a2;
        coeff(axis, 2) = high(0);
        coeff(axis, 1) = high(1);
        coeff(axis, 0) = high(2);
    }

    fastnav::MincoTraj::TrajectoryType global_traj;
    global_traj.emplace_back(duration, coeff);
    global_data_.setGlobalTraj(global_traj, ros::Time::now(), frame_id_);
    global_data_.waypoints_.clear();
    global_data_.waypoints_.push_back(start_pos);
    global_data_.waypoints_.push_back(end_pos);
    last_error_.clear();

    ROS_INFO("[FastNav][LocalPlannerManager] Global reference trajectory generated. duration=%.2f, distance=%.2f",
             duration, distance);
    return true;
}

bool LocalPlannerManager::planToGoal(const Eigen::Vector3d& goal, const ReplanOptions& options)
{
    const auto preempted = [&options]() {
        return options.preempt_requested && options.preempt_requested();
    };

    last_goal_ = goal;
    current_path_.clear();
    last_optimization_result_.clear();
    has_path_ = false;

    if (!has_odom_)
    {
        last_error_ = "No odom available.";
        return false;
    }
    if (!has_map_)
    {
        last_error_ = "No local map available.";
        return false;
    }

    // 起点优先使用 FSM 传入的显式规划起点。
    // 对 GEN_NEW_TRAJ，FSM 会传入当前 odom 状态；对 REPLAN_TRAJ，FSM 会传入旧 MINCO 轨迹在当前执行时刻的
    // $p(t_c),v(t_c),a(t_c)$，这和 EGO-Planner 从当前执行轨迹上取重规划边界条件的原则一致。
    const Eigen::Vector3d start = options.has_start_state ? options.start_pos : currentPosition();
    if (preempted())
    {
        last_error_ = "Planning preempted before A*.";
        return false;
    }

    std::vector<Eigen::Vector3d> raw_path;
    astar_planner_->setCancelCallback(options.preempt_requested);
    const bool success = astar_planner_->plan(start, goal, raw_path);
    astar_planner_->setCancelCallback(std::function<bool()>());
    searched_nodes_cloud_ = centersToCloud(astar_planner_->searchedNodes(), ros::Time::now());

    if (preempted())
    {
        last_error_ = "Planning preempted after A*.";
        return false;
    }

    if (!success)
    {
        last_error_ = astar_planner_->lastError();
        return false;
    }

    size_t preserve_prefix_size = 0;
    const std::vector<Eigen::Vector3d> optimization_reference_path =
        buildOptimizationReferencePath(raw_path, options, preserve_prefix_size);
    if (preempted())
    {
        last_error_ = "Planning preempted after reference path generation.";
        return false;
    }

    fastnav::MincoGcopterOptimizer::BoundaryState start_state;
    start_state.pos = start;
    if (options.has_start_state)
    {
        start_state.vel = options.start_vel;
        start_state.acc = options.start_acc;
    }
    else
    {
        start_state.vel = Eigen::Vector3d(current_odom_.twist.twist.linear.x,
                                          current_odom_.twist.twist.linear.y,
                                          current_odom_.twist.twist.linear.z);
        start_state.acc.setZero();
    }

    fastnav::MincoGcopterOptimizer::BoundaryState goal_state;
    goal_state.pos = goal;
    goal_state.vel = options.goal_vel;
    goal_state.acc = options.goal_acc;

    if (path_optimizer_ && path_optimizer_->optimizeTrajectory(optimization_reference_path,
                                                               voxel_map_,
                                                               start_state,
                                                               goal_state,
                                                               last_optimization_result_,
                                                               preserve_prefix_size,
                                                               options.touch_goal,
                                                               options.preempt_requested))
    {
        if (preempted())
        {
            last_error_ = "Planning preempted after optimization.";
            return false;
        }

        // current_path_ 用于 RViz 和线段碰撞检查；若 MINCO 成功，保存其采样路径，否则保存 shortcut 几何路径。
        current_path_ = last_optimization_result_.sampled_path.empty()
                            ? last_optimization_result_.shortcut_path
                            : last_optimization_result_.sampled_path;
        if (!last_optimization_result_.has_minco && !path_optimizer_->lastError().empty())
        {
            ROS_WARN_THROTTLE(1.0,
                              "[FastNav][LocalPlannerManager] MINCO unavailable, use geometric fallback: %s",
                              path_optimizer_->lastError().c_str());
        }
    }
    else
    {
        last_error_ = path_optimizer_ ? path_optimizer_->lastError() : "PathOptimizer is not set.";
        if (last_error_.empty())
        {
            last_error_ = "Path optimization failed.";
        }
        return false;
    }

    if (current_path_.empty())
    {
        last_error_ = "Optimized path is empty.";
        return false;
    }

    const ros::Time time_now = ros::Time::now();
    // raw_path / shortcut_path 用作几何回退，MINCO 成功时 local_data_ 保存连续可执行轨迹 $p(t)$。
    // global_data_ 是任务级轻量参考轨迹，只由 planGlobalTraj() 维护，不能被局部优化结果覆盖。
    updateTrajInfo(last_optimization_result_, time_now);

    has_path_ = true;
    last_error_.clear();
    return true;
}

// 将 current_path_ 打包为 nav_msgs::Path，供 FSM 发布给 RViz 或后续控制模块。
nav_msgs::Path LocalPlannerManager::getPathMsg() const
{
    nav_msgs::Path path_msg;
    path_msg.header.stamp = ros::Time::now();
    path_msg.header.frame_id = frame_id_;

    path_msg.poses.reserve(current_path_.size());
    for (const Eigen::Vector3d& point : current_path_)
    {
        geometry_msgs::PoseStamped pose;
        pose.header = path_msg.header;
        pose.pose.position.x = point.x();
        pose.pose.position.y = point.y();
        pose.pose.position.z = point.z();
        pose.pose.orientation.w = 1.0;
        path_msg.poses.push_back(pose);
    }

    return path_msg;
}

// 返回当前路径的 Eigen 点列，供同进程内其他模块直接使用。
std::vector<Eigen::Vector3d> LocalPlannerManager::getPath() const
{
    return current_path_;
}

// 外部设置当前路径；目前主要为后续控制 / 轨迹优化扩展预留。
void LocalPlannerManager::setPath(const std::vector<Eigen::Vector3d>& path)
{
    current_path_ = path;
    has_path_ = !current_path_.empty();
    local_data_.reset();
    global_data_.reset();
}

// 返回 A* 搜索过的节点点云，用于 RViz 查看搜索膨胀范围。
sensor_msgs::PointCloud2 LocalPlannerManager::getSearchedNodesCloud() const
{
    return searched_nodes_cloud_;
}

// 返回 planner 内部 VoxelMap 的原始 occupied 体素点云。
sensor_msgs::PointCloud2 LocalPlannerManager::getDebugOccupiedCloud() const
{
    return debug_occupied_cloud_;
}

// 返回 planner 内部 VoxelMap 的 inflated 体素点云；这更接近 A* 实际碰撞查询使用的数据。
sensor_msgs::PointCloud2 LocalPlannerManager::getDebugInflatedCloud() const
{
    return debug_inflated_cloud_;
}

// 检查 current_path_ 的每条线段是否仍然在当前 inflated map 中无碰撞。
bool LocalPlannerManager::isCurrentPathCollisionFree(double step_size) const
{
    if (!has_path_ || current_path_.size() < 2 || !voxel_map_)
    {
        return true;
    }

    for (size_t i = 1; i < current_path_.size(); ++i)
    {
        if (!voxel_map_->isLineFree(current_path_[i - 1], current_path_[i], step_size))
        {
            return false;
        }
    }

    return true;
}

bool LocalPlannerManager::isCurrentTrajectoryCollisionFree(double step_size,
                                                           double check_horizon_ratio,
                                                           bool touch_goal,
                                                           double& collision_time_from_now) const
{
    collision_time_from_now = std::numeric_limits<double>::infinity();
    if (!voxel_map_)
    {
        return true;
    }

    const double ratio = std::min(1.0, std::max(0.05, check_horizon_ratio));
    if (local_data_.valid_ && !local_data_.start_time_.isZero() && local_data_.duration_ > 1.0e-6)
    {
        const ros::Time time_now = ros::Time::now();
        const double t_cur = std::min(local_data_.duration_,
                                      std::max(0.0, local_data_.elapsedTime(time_now)));
        const double t_end = touch_goal ? local_data_.duration_ : local_data_.duration_ * ratio;
        if (t_cur >= t_end)
        {
            return true;
        }

        const double sample_dt = std::max(1.0e-3, step_size);
        Eigen::Vector3d last_pos = local_data_.getPosition(t_cur);
        for (double t = t_cur; t <= t_end + 1.0e-6; t += sample_dt)
        {
            const double t_eval = std::min(t, t_end);
            const Eigen::Vector3d pos = local_data_.getPosition(t_eval);
            if (!voxel_map_->isInMap(pos) ||
                voxel_map_->isInflatedOccupied(pos) ||
                (t_eval > t_cur && !voxel_map_->isLineFree(last_pos, pos, step_size)))
            {
                collision_time_from_now = std::max(0.0, t_eval - t_cur);
                return false;
            }
            last_pos = pos;
        }
        return true;
    }

    if (!has_path_ || current_path_.size() < 2)
    {
        return true;
    }

    // 几何路径回退：若当前没有 MINCO，则只检查采样 path 的前 $2/3$ 或完整 path。
    const size_t check_size = touch_goal ? current_path_.size()
                                         : std::max<size_t>(2, static_cast<size_t>(std::ceil(current_path_.size() * ratio)));
    for (size_t i = 1; i < check_size; ++i)
    {
        if (!voxel_map_->isLineFree(current_path_[i - 1], current_path_[i], step_size))
        {
            collision_time_from_now = static_cast<double>(i) * step_size;
            return false;
        }
    }

    return true;
}

// 从 VoxelMap 读取当前 occupied / inflated 体素中心，并转换为 debug PointCloud2 缓存。
void LocalPlannerManager::updateDebugClouds(const ros::Time& stamp)
{
    debug_occupied_cloud_ = centersToCloud(voxel_map_->getOccupiedVoxelCenters(), stamp);
    debug_inflated_cloud_ = centersToCloud(voxel_map_->getInflatedVoxelCenters(true), stamp);
}

// 将体素中心点或搜索节点从 Eigen 点列转换为 sensor_msgs::PointCloud2。
sensor_msgs::PointCloud2 LocalPlannerManager::centersToCloud(
    const std::vector<Eigen::Vector3d>& centers,
    const ros::Time& stamp) const
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.reserve(centers.size());
    for (const Eigen::Vector3d& center : centers)
    {
        cloud.push_back(pcl::PointXYZ(center.x(), center.y(), center.z()));
    }
    cloud.width = cloud.size();
    cloud.height = 1;
    cloud.is_dense = true;

    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(cloud, msg);
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id_;
    return msg;
}

std::vector<Eigen::Vector3d> LocalPlannerManager::buildOptimizationReferencePath(
    const std::vector<Eigen::Vector3d>& raw_path,
    const ReplanOptions& options,
    size_t& preserve_prefix_size) const
{
    preserve_prefix_size = 0;
    std::vector<Eigen::Vector3d> reference_path =
        options.use_current_traj ? buildCurrentTrajReferencePath(raw_path, options, preserve_prefix_size) : raw_path;

    if (!options.use_random_init || reference_path.size() < 2 || !voxel_map_ || random_init_scale_ <= 1.0e-6)
    {
        return reference_path;
    }

    // 若 reference_path 前缀来自旧 MINCO 剩余段，则随机扰动只能作用于后续桥接段。
    // 这样连续失败时仍能产生不同初始化，但不会破坏旧轨迹前缀 $p_{old}(t_c..)$。
    const size_t mutable_begin = std::min(preserve_prefix_size, reference_path.size() - 1);
    if (mutable_begin + 1 >= reference_path.size())
    {
        return reference_path;
    }

    // EGO-v2 在连续失败后会扰动初始多项式的中间点。
    // FastNav 使用 A* path 生成 safe corridor，因此这里扰动路径中点 $r_m$，让 FIRI/MINCO 得到不同的初始走廊。
    const Eigen::Vector3d start = reference_path.front();
    const Eigen::Vector3d goal = reference_path.back();
    const double path_span = std::max(0.5, (goal - start).norm());

    const bool has_middle_point = reference_path.size() > mutable_begin + 2;
    const size_t mid_id = has_middle_point ? mutable_begin + (reference_path.size() - mutable_begin) / 2 : mutable_begin;
    const Eigen::Vector3d prev = has_middle_point ? reference_path[mid_id - 1] : start;
    const Eigen::Vector3d next = has_middle_point ? reference_path[mid_id + 1] : goal;
    const Eigen::Vector3d center = has_middle_point ? reference_path[mid_id] : 0.5 * (start + goal);

    Eigen::Vector3d dir = next - prev;
    if (dir.norm() < 1.0e-6)
    {
        dir = goal - start;
    }
    if (dir.norm() < 1.0e-6)
    {
        return reference_path;
    }
    dir.normalize();

    Eigen::Vector3d basis1 = dir.cross(Eigen::Vector3d::UnitZ());
    if (basis1.norm() < 1.0e-3)
    {
        basis1 = dir.cross(Eigen::Vector3d::UnitY());
    }
    basis1.normalize();
    Eigen::Vector3d basis2 = dir.cross(basis1).normalized();

    const double failure_gain = 1.0 + 0.25 * std::max(0, options.continuous_failures);
    const double base_radius = std::min(3.0, std::max(voxel_map_->resolution(),
                                                      random_init_scale_ * failure_gain * path_span * 0.15));
    const int seed = std::max(0, options.attempt) + std::max(0, options.continuous_failures);

    for (int i = 0; i < 8; ++i)
    {
        const double angle = 0.7853981633974483 * static_cast<double>(seed + i);
        const double radius = base_radius * (1.0 + 0.2 * static_cast<double>(i / 2));
        const Eigen::Vector3d candidate =
            center + radius * (std::cos(angle) * basis1 + std::sin(angle) * basis2);

        if (!voxel_map_->isInMap(candidate) || voxel_map_->isInflatedOccupied(candidate))
        {
            continue;
        }
        if (!voxel_map_->isLineFree(prev, candidate, astar_config_.line_check_step) ||
            !voxel_map_->isLineFree(candidate, next, astar_config_.line_check_step))
        {
            continue;
        }

        if (has_middle_point)
        {
            reference_path[mid_id] = candidate;
        }
        else
        {
            const size_t insert_id = preserve_prefix_size > 0 ? mutable_begin : 1;
            reference_path.insert(reference_path.begin() + static_cast<std::ptrdiff_t>(insert_id), candidate);
        }
        return reference_path;
    }

    return reference_path;
}

std::vector<Eigen::Vector3d> LocalPlannerManager::buildCurrentTrajReferencePath(
    const std::vector<Eigen::Vector3d>& raw_path,
    const ReplanOptions& options,
    size_t& preserve_prefix_size) const
{
    preserve_prefix_size = 0;
    if (raw_path.size() < 2 || !voxel_map_ || !local_data_.valid_ || local_data_.duration_ <= 1.0e-6)
    {
        return raw_path;
    }

    const ros::Time time_now = ros::Time::now();
    const double t_cur = std::min(local_data_.duration_,
                                  std::max(0.0, local_data_.elapsedTime(time_now)));
    const double sample_dt = std::max(0.05, minco_sample_dt_);
    const double min_dist = std::max(0.5 * voxel_map_->resolution(), 1.0e-3);

    std::vector<Eigen::Vector3d> old_segment;
    old_segment.reserve(static_cast<size_t>(std::ceil((local_data_.duration_ - t_cur) / sample_dt)) + 2);

    Eigen::Vector3d last = options.has_start_state ? options.start_pos : local_data_.getPosition(t_cur);
    if (!voxel_map_->isInMap(last) || voxel_map_->isInflatedOccupied(last))
    {
        return raw_path;
    }
    old_segment.push_back(last);

    // 从当前旧 MINCO 轨迹时间 $t_c$ 向后采样，只保留仍在地图内且不穿过 inflated voxel 的安全剩余段。
    // 一旦旧轨迹前方被新障碍截断，就停止复用，后续交给 A* 桥接到目标。
    for (double t = t_cur + sample_dt; t <= local_data_.duration_ + 1.0e-4; t += sample_dt)
    {
        const Eigen::Vector3d pos = local_data_.getPosition(std::min(t, local_data_.duration_));
        if (!voxel_map_->isInMap(pos) ||
            voxel_map_->isInflatedOccupied(pos) ||
            !voxel_map_->isLineFree(last, pos, astar_config_.line_check_step))
        {
            break;
        }

        if ((pos - old_segment.back()).norm() > min_dist)
        {
            old_segment.push_back(pos);
            last = pos;
        }
    }

    if (old_segment.size() < 2)
    {
        return raw_path;
    }

    const Eigen::Vector3d bridge_start = old_segment.back();
    int bridge_id = -1;
    double best_dist = std::numeric_limits<double>::infinity();
    for (int i = 0; i < static_cast<int>(raw_path.size()); ++i)
    {
        if (!voxel_map_->isLineFree(bridge_start, raw_path[i], astar_config_.line_check_step))
        {
            continue;
        }

        const double dist = (bridge_start - raw_path[i]).norm();
        if (dist < best_dist)
        {
            best_dist = dist;
            bridge_id = i;
        }
    }

    if (bridge_id < 0)
    {
        return raw_path;
    }

    std::vector<Eigen::Vector3d> reference_path = old_segment;
    for (int i = bridge_id; i < static_cast<int>(raw_path.size()); ++i)
    {
        if ((raw_path[i] - reference_path.back()).norm() <= min_dist)
        {
            continue;
        }
        reference_path.push_back(raw_path[i]);
    }

    if ((reference_path.back() - raw_path.back()).norm() > min_dist)
    {
        reference_path.push_back(raw_path.back());
    }

    if (reference_path.size() < 2)
    {
        return raw_path;
    }

    preserve_prefix_size = old_segment.size();
    return reference_path;
}

void LocalPlannerManager::updateTrajInfo(const PathOptimizer::OptimizationResult& result,
                                         const ros::Time& time_now)
{
    local_data_.reset();
    if (!result.has_minco || !result.minco_traj.valid())
    {
        return;
    }

    // LocalTrajData 保存当前可执行 MINCO 轨迹 $p(t)$，并额外缓存采样点用于 RViz / path topic。
    local_data_.setLocalTraj(result.minco_traj.trajectory(),
                             time_now,
                             frame_id_,
                             minco_sample_dt_);
    local_data_.corridor_ = result.corridors;
    if (!result.sampled_path.empty())
    {
        local_data_.sampled_path_ = result.sampled_path;
    }
}

// 从 current_odom_ 中提取当前位置向量，作为局部地图中心和规划起点。
Eigen::Vector3d LocalPlannerManager::currentPosition() const
{
    return Eigen::Vector3d(current_odom_.pose.pose.position.x,
                           current_odom_.pose.pose.position.y,
                           current_odom_.pose.pose.position.z);
}

}  // namespace fastnav_planner
