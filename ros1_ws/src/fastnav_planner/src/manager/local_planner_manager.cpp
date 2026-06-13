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
#include <ros/time.h>

namespace fastnav_planner
{

namespace
{

double smoothReferenceDuration(double distance, double max_vel, double max_acc)
{
    if (distance < 1.0e-4)
    {
        return 0.0;
    }

    // 五次多项式参考段的峰值速度/加速度会高于 $d/T$ 与 $d/T^2$，
    // 这里使用保守时间 $T=max(2d/v_{max}, sqrt(6d/a_{max}))$。
    return std::max(0.5,
                    std::max(2.0 * distance / std::max(0.1, max_vel),
                             std::sqrt(6.0 * distance / std::max(0.1, max_acc))));
}

Piece<5> makeQuinticPiece(const Eigen::Vector3d& p0,
                          const Eigen::Vector3d& v0,
                          const Eigen::Vector3d& a0,
                          const Eigen::Vector3d& p1,
                          const Eigen::Vector3d& v1,
                          const Eigen::Vector3d& a1,
                          double duration)
{
    const double T = std::max(1.0e-3, duration);
    Eigen::Matrix3d A;
    A << std::pow(T, 3), std::pow(T, 4), std::pow(T, 5),
         3.0 * std::pow(T, 2), 4.0 * std::pow(T, 3), 5.0 * std::pow(T, 4),
         6.0 * T, 12.0 * std::pow(T, 2), 20.0 * std::pow(T, 3);

    Piece<5>::CoefficientMat coeff;
    coeff.setZero();
    for (int axis = 0; axis < 3; ++axis)
    {
        const double c0 = p0(axis);
        const double c1 = v0(axis);
        const double c2 = 0.5 * a0(axis);

        Eigen::Vector3d b;
        b(0) = p1(axis) - (c0 + c1 * T + c2 * T * T);
        b(1) = v1(axis) - (c1 + 2.0 * c2 * T);
        b(2) = a1(axis) - (2.0 * c2);

        const Eigen::Vector3d high = A.lu().solve(b);
        // GCOPTER Piece<5> 的系数列顺序为 $[t^5,t^4,t^3,t^2,t,1]$。
        coeff(axis, 5) = c0;
        coeff(axis, 4) = c1;
        coeff(axis, 3) = c2;
        coeff(axis, 2) = high(0);
        coeff(axis, 1) = high(1);
        coeff(axis, 0) = high(2);
    }

    return Piece<5>(T, coeff);
}

std::string searchStatusName(AStarPlanner::SearchStatus status)
{
    switch (status)
    {
    case AStarPlanner::SearchStatus::SUCCESS:
        return "SUCCESS";
    case AStarPlanner::SearchStatus::REACH_GOAL:
        return "REACH_GOAL";
    case AStarPlanner::SearchStatus::REACH_HORIZON:
        return "REACH_HORIZON";
    case AStarPlanner::SearchStatus::BEST_EFFORT:
        return "BEST_EFFORT";
    case AStarPlanner::SearchStatus::TIME_OUT:
        return "TIME_OUT";
    case AStarPlanner::SearchStatus::NO_PATH:
        return "NO_PATH";
    case AStarPlanner::SearchStatus::INIT_ERROR:
        return "INIT_ERROR";
    case AStarPlanner::SearchStatus::PREEMPTED:
        return "PREEMPTED";
    }
    return "UNKNOWN";
}

void prependPathPrefix(const std::vector<Eigen::Vector3d>& prefix,
                       std::vector<Eigen::Vector3d>& path)
{
    if (prefix.size() < 2)
    {
        return;
    }

    std::vector<Eigen::Vector3d> merged;
    merged.reserve(prefix.size() + path.size());
    for (const Eigen::Vector3d& point : prefix)
    {
        if (merged.empty() || (point - merged.back()).norm() > 1.0e-4)
        {
            merged.push_back(point);
        }
    }
    for (const Eigen::Vector3d& point : path)
    {
        if (merged.empty() || (point - merged.back()).norm() > 1.0e-4)
        {
            merged.push_back(point);
        }
    }
    path.swap(merged);
}

}  // namespace

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
    frontend_voxel_map_ = std::make_shared<fastnav_mapping::VoxelMap>();
    frontend_voxel_map_->init(resolution_,
                              local_x_size_,
                              local_y_size_,
                              local_z_size_,
                              local_z_min_,
                              local_z_max_,
                              drone_radius_,
                              safety_margin_ + astar_config_.min_clearance);
    // planner 内部 VoxelMap 不再每帧完全重建，而是像 EGO-Planner 的 GridMap 一样长期持有局部缓存。
    // 点云命中用 log-odds 累计 $l_i <- clamp(l_i + log(p_hit/(1-p_hit)), l_min, l_max)$；
    // 当前版本暂不做 raycasting free-space，因此用 temporal_decay_log 把旧障碍缓慢拉回未知 $l=0$。
    voxel_map_->setOccupancyUpdateParams(map_p_hit_,
                                         map_p_miss_,
                                         map_p_min_,
                                         map_p_max_,
                                         map_p_occ_,
                                         map_temporal_decay_log_);
    frontend_voxel_map_->setOccupancyUpdateParams(map_p_hit_,
                                                  map_p_miss_,
                                                  map_p_min_,
                                                  map_p_max_,
                                                  map_p_occ_,
                                                  map_temporal_decay_log_);

    astar_planner_ = std::make_shared<AStarPlanner>();
    astar_planner_->setMap(frontend_voxel_map_);
    astar_planner_->setConfig(astar_config_);

    path_optimizer_ = std::make_shared<PathOptimizer>();
    path_optimizer_->setConfig(optimizer_config_);
    debug_recorder_config_.frame_id = frame_id_;
    debug_recorder_config_.minco_sample_dt = minco_sample_dt_;
    debug_recorder_.setConfig(debug_recorder_config_);

    global_data_.reset();
    local_data_.reset();

    ROS_INFO("[FastNav][LocalPlannerManager] Initialized. resolution=%.3f, frame=%s, base_inflation=%.3f, frontend_inflation=%.3f",
             resolution_,
             frame_id_.c_str(),
             voxel_map_->inflationRadius(),
             frontend_voxel_map_->inflationRadius());
    if (debug_recorder_.enabled())
    {
        ROS_INFO("[FastNav][LocalPlannerManager] Failure snapshot recording enabled: %s",
                 debug_recorder_config_.output_dir.c_str());
    }
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
    nh.param<double>("/local_planner/astar/clearance_retry_scale", astar_config_.clearance_retry_scale, astar_config_.clearance_retry_scale);
    nh.param<double>("/local_planner/astar/min_clearance_floor", astar_config_.min_clearance_floor, astar_config_.min_clearance_floor);
    nh.param<double>("/local_planner/astar/line_check_step", astar_config_.line_check_step, astar_config_.line_check_step);
    nh.param<bool>("/local_planner/astar/enable_guide_path_reuse", enable_guide_path_reuse_, enable_guide_path_reuse_);
    nh.param<double>("/local_planner/astar/guide_reuse_start_tolerance", guide_reuse_start_tolerance_, guide_reuse_start_tolerance_);
    nh.param<double>("/local_planner/astar/guide_reuse_goal_tolerance", guide_reuse_goal_tolerance_, guide_reuse_goal_tolerance_);
    nh.param<int>("/local_planner/astar/max_search_nodes", astar_config_.max_search_nodes, astar_config_.max_search_nodes);
    nh.param<double>("/local_planner/astar/max_search_time", astar_config_.max_search_time, astar_config_.max_search_time);
    nh.param<bool>("/local_planner/astar/enable_goal_projection", astar_config_.enable_goal_projection, astar_config_.enable_goal_projection);
    nh.param<int>("/local_planner/astar/projection_margin_voxels", astar_config_.projection_margin_voxels, astar_config_.projection_margin_voxels);
    nh.param<double>("/local_planner/astar/nearest_free_search_radius", astar_config_.nearest_free_search_radius, astar_config_.nearest_free_search_radius);
    nh.param<bool>("/local_planner/astar/allow_timeout_best_effort", astar_config_.allow_timeout_best_effort, astar_config_.allow_timeout_best_effort);
    nh.param<double>("/local_planner/astar/timeout_best_effort_min_length", astar_config_.timeout_best_effort_min_length, astar_config_.timeout_best_effort_min_length);
    nh.param<double>("/local_planner/astar/timeout_horizon_scale", astar_config_.timeout_horizon_scale, astar_config_.timeout_horizon_scale);
    nh.param<double>("/local_planner/astar/timeout_min_horizon", astar_config_.timeout_min_horizon, astar_config_.timeout_min_horizon);
    nh.param<bool>("/local_planner/astar/enable_escape_search", astar_config_.enable_escape_search, astar_config_.enable_escape_search);
    nh.param<double>("/local_planner/astar/escape_max_radius", astar_config_.escape_max_radius, astar_config_.escape_max_radius);
    nh.param<double>("/local_planner/astar/escape_max_search_time", astar_config_.escape_max_search_time, astar_config_.escape_max_search_time);
    nh.param<int>("/local_planner/astar/escape_max_nodes", astar_config_.escape_max_nodes, astar_config_.escape_max_nodes);
    nh.param<double>("/planner/astar/heuristic_weight", astar_config_.heuristic_weight, astar_config_.heuristic_weight);
    nh.param<double>("/planner/astar/min_clearance", astar_config_.min_clearance, astar_config_.min_clearance);
    nh.param<double>("/planner/astar/clearance_retry_scale", astar_config_.clearance_retry_scale, astar_config_.clearance_retry_scale);
    nh.param<double>("/planner/astar/min_clearance_floor", astar_config_.min_clearance_floor, astar_config_.min_clearance_floor);
    nh.param<int>("/planner/astar/max_search_nodes", astar_config_.max_search_nodes, astar_config_.max_search_nodes);
    nh.param<double>("/planner/astar/max_search_time", astar_config_.max_search_time, astar_config_.max_search_time);
    nh.param<bool>("/planner/astar/enable_goal_projection", astar_config_.enable_goal_projection, astar_config_.enable_goal_projection);
    nh.param<int>("/planner/astar/projection_margin_voxels", astar_config_.projection_margin_voxels, astar_config_.projection_margin_voxels);
    nh.param<double>("/planner/astar/nearest_free_search_radius", astar_config_.nearest_free_search_radius, astar_config_.nearest_free_search_radius);
    nh.param<bool>("/planner/astar/allow_timeout_best_effort", astar_config_.allow_timeout_best_effort, astar_config_.allow_timeout_best_effort);
    nh.param<double>("/planner/astar/timeout_best_effort_min_length", astar_config_.timeout_best_effort_min_length, astar_config_.timeout_best_effort_min_length);
    nh.param<double>("/planner/astar/timeout_horizon_scale", astar_config_.timeout_horizon_scale, astar_config_.timeout_horizon_scale);
    nh.param<double>("/planner/astar/timeout_min_horizon", astar_config_.timeout_min_horizon, astar_config_.timeout_min_horizon);
    nh.param<bool>("/planner/astar/enable_escape_search", astar_config_.enable_escape_search, astar_config_.enable_escape_search);
    nh.param<double>("/planner/astar/escape_max_radius", astar_config_.escape_max_radius, astar_config_.escape_max_radius);
    nh.param<double>("/planner/astar/escape_max_search_time", astar_config_.escape_max_search_time, astar_config_.escape_max_search_time);
    nh.param<int>("/planner/astar/escape_max_nodes", astar_config_.escape_max_nodes, astar_config_.escape_max_nodes);
    nh.param<bool>("/planner/astar/enable_guide_path_reuse", enable_guide_path_reuse_, enable_guide_path_reuse_);
    nh.param<double>("/planner/astar/guide_reuse_start_tolerance", guide_reuse_start_tolerance_, guide_reuse_start_tolerance_);
    nh.param<double>("/planner/astar/guide_reuse_goal_tolerance", guide_reuse_goal_tolerance_, guide_reuse_goal_tolerance_);
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
    pnh.param<double>("astar/clearance_retry_scale", astar_config_.clearance_retry_scale, astar_config_.clearance_retry_scale);
    pnh.param<double>("astar/min_clearance_floor", astar_config_.min_clearance_floor, astar_config_.min_clearance_floor);
    pnh.param<double>("astar/max_search_time", astar_config_.max_search_time, astar_config_.max_search_time);
    pnh.param<bool>("astar/enable_goal_projection", astar_config_.enable_goal_projection, astar_config_.enable_goal_projection);
    pnh.param<int>("astar/projection_margin_voxels", astar_config_.projection_margin_voxels, astar_config_.projection_margin_voxels);
    pnh.param<double>("astar/nearest_free_search_radius", astar_config_.nearest_free_search_radius, astar_config_.nearest_free_search_radius);
    pnh.param<bool>("astar/allow_timeout_best_effort", astar_config_.allow_timeout_best_effort, astar_config_.allow_timeout_best_effort);
    pnh.param<double>("astar/timeout_best_effort_min_length", astar_config_.timeout_best_effort_min_length, astar_config_.timeout_best_effort_min_length);
    pnh.param<double>("astar/timeout_horizon_scale", astar_config_.timeout_horizon_scale, astar_config_.timeout_horizon_scale);
    pnh.param<double>("astar/timeout_min_horizon", astar_config_.timeout_min_horizon, astar_config_.timeout_min_horizon);
    pnh.param<bool>("astar/enable_escape_search", astar_config_.enable_escape_search, astar_config_.enable_escape_search);
    pnh.param<double>("astar/escape_max_radius", astar_config_.escape_max_radius, astar_config_.escape_max_radius);
    pnh.param<double>("astar/escape_max_search_time", astar_config_.escape_max_search_time, astar_config_.escape_max_search_time);
    pnh.param<int>("astar/escape_max_nodes", astar_config_.escape_max_nodes, astar_config_.escape_max_nodes);
    pnh.param<bool>("astar/enable_guide_path_reuse", enable_guide_path_reuse_, enable_guide_path_reuse_);
    pnh.param<double>("astar/guide_reuse_start_tolerance", guide_reuse_start_tolerance_, guide_reuse_start_tolerance_);
    pnh.param<double>("astar/guide_reuse_goal_tolerance", guide_reuse_goal_tolerance_, guide_reuse_goal_tolerance_);
    pnh.param<bool>("optimizer/enable", optimizer_config_.enable, optimizer_config_.enable);
    pnh.param<bool>("optimizer/shortcut", optimizer_config_.shortcut, optimizer_config_.shortcut);
    pnh.param<double>("optimizer/line_check_step", optimizer_config_.line_check_step, optimizer_config_.line_check_step);
    pnh.param<bool>("optimizer/minco_enable", optimizer_config_.minco_enable, optimizer_config_.minco_enable);
    pnh.param<bool>("optimizer/require_minco", optimizer_config_.require_minco, optimizer_config_.require_minco);
    pnh.param<double>("optimizer/minco_sample_dt", optimizer_config_.minco_sample_dt, optimizer_config_.minco_sample_dt);
    pnh.param<int>("optimizer/max_retry", optimizer_config_.max_retry, optimizer_config_.max_retry);
    pnh.param<double>("optimizer/random_init_scale", random_init_scale_, random_init_scale_);

    bool debug_enable = false;
    bool record_failure_cases = false;
    nh.param<bool>("/local_planner/debug/enable", debug_enable, debug_enable);
    nh.param<bool>("/local_planner/debug/record_failure_cases", record_failure_cases, record_failure_cases);
    nh.param<std::string>("/local_planner/debug/failure_case_dir", debug_recorder_config_.output_dir, debug_recorder_config_.output_dir);
    nh.param<int>("/local_planner/debug/max_failure_cases", debug_recorder_config_.max_cases, debug_recorder_config_.max_cases);
    nh.param<bool>("/local_planner/debug/record_cloud", debug_recorder_config_.record_cloud, debug_recorder_config_.record_cloud);
    nh.param<bool>("/local_planner/debug/record_voxel_map", debug_recorder_config_.record_voxel_map, debug_recorder_config_.record_voxel_map);
    nh.param<bool>("/local_planner/debug/record_corridor", debug_recorder_config_.record_corridor, debug_recorder_config_.record_corridor);
    nh.param<bool>("/local_planner/debug/record_minco_samples", debug_recorder_config_.record_minco_samples, debug_recorder_config_.record_minco_samples);
    pnh.param<bool>("debug/enable", debug_enable, debug_enable);
    pnh.param<bool>("debug/record_failure_cases", record_failure_cases, record_failure_cases);
    pnh.param<std::string>("debug/failure_case_dir", debug_recorder_config_.output_dir, debug_recorder_config_.output_dir);
    pnh.param<int>("debug/max_failure_cases", debug_recorder_config_.max_cases, debug_recorder_config_.max_cases);
    pnh.param<bool>("debug/record_cloud", debug_recorder_config_.record_cloud, debug_recorder_config_.record_cloud);
    pnh.param<bool>("debug/record_voxel_map", debug_recorder_config_.record_voxel_map, debug_recorder_config_.record_voxel_map);
    pnh.param<bool>("debug/record_corridor", debug_recorder_config_.record_corridor, debug_recorder_config_.record_corridor);
    pnh.param<bool>("debug/record_minco_samples", debug_recorder_config_.record_minco_samples, debug_recorder_config_.record_minco_samples);

    minco_sample_dt_ = std::max(1.0e-3, optimizer_config_.minco_sample_dt);
    debug_recorder_config_.enable = debug_enable && record_failure_cases;
    debug_recorder_config_.minco_sample_dt = minco_sample_dt_;
    debug_recorder_config_.max_cases = std::max(0, debug_recorder_config_.max_cases);
    random_init_scale_ = std::max(0.0, random_init_scale_);
    astar_config_.min_clearance = std::max(0.0, astar_config_.min_clearance);
    astar_config_.min_clearance_floor = std::max(0.0, std::min(astar_config_.min_clearance_floor, astar_config_.min_clearance));
    astar_config_.clearance_retry_scale = std::min(0.99, std::max(0.05, astar_config_.clearance_retry_scale));
    astar_config_.max_search_time = std::max(0.0, astar_config_.max_search_time);
    astar_config_.projection_margin_voxels = std::max(0, astar_config_.projection_margin_voxels);
    astar_config_.nearest_free_search_radius = std::max(0.0, astar_config_.nearest_free_search_radius);
    astar_config_.timeout_best_effort_min_length = std::max(0.0, astar_config_.timeout_best_effort_min_length);
    astar_config_.timeout_horizon_scale = std::min(0.95, std::max(0.05, astar_config_.timeout_horizon_scale));
    astar_config_.timeout_min_horizon = std::max(0.0, astar_config_.timeout_min_horizon);
    astar_config_.escape_max_radius = std::max(0.0, astar_config_.escape_max_radius);
    astar_config_.escape_max_search_time = std::max(0.0, astar_config_.escape_max_search_time);
    astar_config_.escape_max_nodes = std::max(1, astar_config_.escape_max_nodes);
    guide_reuse_start_tolerance_ = std::max(resolution_, guide_reuse_start_tolerance_);
    guide_reuse_goal_tolerance_ = std::max(resolution_, guide_reuse_goal_tolerance_);
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

    latest_filtered_cloud_ = *msg;

    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);

    // 局部地图中心跟随无人机位置 $c$ 滑动，VoxelMap 会按整格偏移搬运旧缓存。
    // 因此这里不再 reset；旧障碍若持续被看见会继续累积，没被看见则通过 $l_i -> 0$ 的衰减逐渐遗忘。
    voxel_map_->setMapCenter(currentPosition());
    frontend_voxel_map_->setMapCenter(currentPosition());
    voxel_map_->decayOccupancy();
    frontend_voxel_map_->decayOccupancy();

    for (const pcl::PointXYZ& point : cloud.points)
    {
        const Eigen::Vector3d pos(point.x, point.y, point.z);
        voxel_map_->setOccupied(pos);
        frontend_voxel_map_->setOccupied(pos);
    }
    voxel_map_->inflateObstacles();
    frontend_voxel_map_->inflateObstacles();

    last_cloud_stamp_ = msg->header.stamp;
    has_map_ = true;
    updateDebugClouds(msg->header.stamp);
}

// 默认规划入口：只给定目标点 $p_g$，使用默认 ReplanOptions。
// 这个重载主要用于简单测试或外部模块不关心重规划细节的场景；
// 它不会复制一套规划逻辑，而是把默认参数转发给完整版本 planToGoal(goal, options)。
bool LocalPlannerManager::planToGoal(const Eigen::Vector3d& goal)
{
    ReplanOptions options;
    return planToGoal(goal, options);
}

// 生成任务级全局参考轨迹 global_data_。
// 这个函数不是最终局部避障优化器：它只给 PlannerFSM::getLocalTarget() 提供一条可查询的参考曲线 $p_g(t)$。
// 真正用于飞行的局部 MINCO 轨迹由 planToGoal() 内部的 A* -> safe corridor -> PathOptimizer 生成。
// 当前全局参考采用轻量策略：先用 guide A* 在 inflated map 中找一个 horizon 内的 local target，
// 再用一段或两段五次多项式连接 $p_s$、$p_{local}$、$p_{end}$。
bool LocalPlannerManager::planGlobalTraj(const Eigen::Vector3d& start_pos,
                                         const Eigen::Vector3d& start_vel,
                                         const Eigen::Vector3d& start_acc,
                                         const Eigen::Vector3d& end_pos,
                                         const Eigen::Vector3d& end_vel,
                                         const Eigen::Vector3d& end_acc,
                                         double planning_horizon)
{
    // 任务方向向量 $\\Delta p=p_{end}-p_s$ 和任务距离 $d=\\|\\Delta p\\|$。
    // 若距离过小，FSM 会直接认为目标已在附近，没有必要构造全局参考轨迹。
    const Eigen::Vector3d delta = end_pos - start_pos;
    const double distance = delta.norm();
    if (distance < 1.0e-4)
    {
        last_error_ = "Global trajectory start and goal are too close.";
        return false;
    }

    // 全局参考的时间分配只使用动力学上限做保守估计，不做复杂优化。
    // smoothReferenceDuration() 内部近似使用 $T=max(2d/v_{max},\\sqrt{6d/a_{max}})$。
    const double max_vel = std::max(0.1, optimizer_config_.feasibility.max_vel);
    const double max_acc = std::max(0.1, optimizer_config_.feasibility.max_acc);
    const double horizon = planning_horizon > 1.0e-3 ? planning_horizon : distance;

    std::vector<Eigen::Vector3d> guide_path;
    Eigen::Vector3d local_target = end_pos;
    Eigen::Vector3d local_dir = delta / std::max(distance, 1.0e-6);
    bool guide_success = false;
    double guide_clearance_used = astar_config_.min_clearance;

    // guide path 只对当前 global reference 有效。每次生成新全局参考时先清空旧缓存，
    // 只有 guide A* 成功后才重新写入，避免 planToGoal() 复用过期拓扑。
    cached_guide_path_.clear();
    cached_guide_clearance_used_ = 0.0;
    cached_guide_stamp_ = ros::Time();

    if (has_map_ && astar_planner_ && distance > horizon + 1.0e-3)
    {
        // 目标超过 planning horizon 时，不再简单取直线方向上的 $p_s+H\\hat d$。
        // runGuideAStarWithFallback() 会在 frontend map / base map 中搜索一段长度约为 $H$ 的可通行路径，
        // 其末端作为 local target，使全局参考的第一段方向更贴近障碍环境。
        const ros::WallTime guide_start = ros::WallTime::now();
        int guide_expanded_nodes = 0;
        guide_success = runGuideAStarWithFallback(start_pos,
                                                  end_pos,
                                                  horizon,
                                                  guide_path,
                                                  guide_clearance_used,
                                                  guide_expanded_nodes);
        last_timing_.guide_astar_ms = (ros::WallTime::now() - guide_start).toSec() * 1000.0;
        searched_nodes_cloud_ = centersToCloud(astar_planner_->searchedNodes(), ros::Time::now());
        if (guide_success && guide_path.size() >= 2)
        {
            // guide_path.back() 是 A* 沿最终目标方向推进到 horizon 后得到的局部目标 $p_{local}$。
            // local_dir 取最后一段方向，用于给中间点速度 $v_{local}$ 赋方向。
            local_target = guide_path.back();
            local_dir = guide_path.back() - guide_path[guide_path.size() - 2];
            if (local_dir.norm() < 1.0e-6)
            {
                local_dir = local_target - start_pos;
            }
            if (local_dir.norm() < 1.0e-6)
            {
                local_dir = delta;
            }
            local_dir.normalize();

            // 这条 guide path 后续可直接作为 start -> local_target 的 raw_path。
            // 复用前仍会检查起终点偏差和当前地图碰撞，因此这里只做缓存，不直接承诺可执行。
            cached_guide_path_ = guide_path;
            cached_guide_clearance_used_ = guide_clearance_used;
            cached_guide_stamp_ = ros::Time::now();
        }
        else
        {
            ROS_WARN_THROTTLE(1.0,
                              "[FastNav][LocalPlannerManager] Guide A* failed, fallback to direct global reference: %s",
                              last_error_.c_str());
        }
    }

    if (!guide_success && distance > horizon + 1.0e-3)
    {
        // guide A* 不可用或失败时，退回直线 horizon 切片：$p_{local}=p_s+H\\hat d$。
        // 这保证全局参考仍然可生成，后续局部 A* / MINCO 会继续做真实避障约束。
        local_target = start_pos + local_dir * horizon;
        ROS_WARN_THROTTLE(1.0,"[FastNav][LocalPlannerManager] Guide A* failed, fallback to direct local target: [%.2f, %.2f, %.2f]",
                          local_target.x(), local_target.y(), local_target.z());
    }

    fastnav::MincoTraj::TrajectoryType global_traj;
    global_data_.waypoints_.clear();
    global_data_.waypoints_.push_back(start_pos);

    const bool local_target_is_goal = (local_target - end_pos).norm() < std::max(0.2, resolution_);
    if (local_target_is_goal)
    {
        // 若 local target 已经等价于最终目标，则只用一段五次多项式。
        // makeQuinticPiece() 解的是边界约束 $p(0),v(0),a(0),p(T),v(T),a(T)$，
        // 因此该参考段在位置、速度、加速度上都满足端点条件。
        const double duration = smoothReferenceDuration(distance, max_vel, max_acc);
        global_traj.emplace_back(makeQuinticPiece(start_pos,
                                                  start_vel,
                                                  start_acc,
                                                  end_pos,
                                                  end_vel,
                                                  end_acc,
                                                  duration));
    }
    else
    {
        const double d1 = std::max(1.0e-4, (local_target - start_pos).norm());
        const double d2 = std::max(1.0e-4, (end_pos - local_target).norm());
        const double t1 = smoothReferenceDuration(d1, max_vel, max_acc);
        const double t2 = smoothReferenceDuration(d2, max_vel, max_acc);

        // 两段参考曲线分别连接 $p_s -> p_{local}$ 和 $p_{local} -> p_{end}$。
        // 中间点速度沿 guide A* 最后一段方向，大小用两段平均速度估计：
        // $v_m = \\hat d_{guide} min(v_{max}, max(0.1, 0.5(d_1/t_1+d_2/t_2)))$。
        // 中间加速度设为 0，让两段五次多项式在 $p,v,a$ 上连续。
        const double mid_speed = std::min(max_vel, std::max(0.1, 0.5 * (d1 / std::max(t1, 1.0e-3) +
                                                                        d2 / std::max(t2, 1.0e-3))));
        const Eigen::Vector3d mid_vel = local_dir * mid_speed;
        const Eigen::Vector3d mid_acc = Eigen::Vector3d::Zero();

        global_traj.emplace_back(makeQuinticPiece(start_pos,
                                                  start_vel,
                                                  start_acc,
                                                  local_target,
                                                  mid_vel,
                                                  mid_acc,
                                                  t1));
        global_traj.emplace_back(makeQuinticPiece(local_target,
                                                  mid_vel,
                                                  mid_acc,
                                                  end_pos,
                                                  end_vel,
                                                  end_acc,
                                                  t2));
        global_data_.waypoints_.push_back(local_target);
    }
    global_data_.waypoints_.push_back(end_pos);

    // global_data_ 保存的是任务级参考轨迹和参考航点，FSM 后续会沿 $p_g(t)$ 搜索局部目标。
    // 它不会直接发送给控制器，也不会替代 local_data_ 中的可执行 MINCO 轨迹。
    global_data_.setGlobalTraj(global_traj, ros::Time::now(), frame_id_);
    last_error_.clear();

    ROS_INFO("[FastNav][LocalPlannerManager] Global reference trajectory generated. duration=%.2f, distance=%.2f, guide=%d, local_target=[%.2f, %.2f, %.2f]",
             global_data_.duration_,
             distance,
             guide_success,
             local_target.x(),
             local_target.y(),
             local_target.z());
    return true;
}

// 完整规划入口：从指定起点状态规划到目标点 $p_g$，并更新 manager 内部的 local_data_。
// 输入 goal 在 FSM 语义上通常是 local_target，而不一定是任务最终 end_pt_。
// 整体链路为：
// 1. 根据 ReplanOptions 选取起点边界 $p_0,v_0,a_0$；
// 2. runAStarWithFallback() 生成几何 raw_path，包含 frontend map、base map、escape、best-effort 等前端策略；
// 3. buildOptimizationReferencePath() 将 raw_path 转成 MINCO/corridor 初始化路径；
// 4. PathOptimizer 生成 safe corridor，并调用 GCOPTER/MINCO 优化；
// 5. fine check 通过后，把可执行轨迹保存到 local_data_，把 RViz 采样路径保存到 current_path_。
bool LocalPlannerManager::planToGoal(const Eigen::Vector3d& goal, const ReplanOptions& options)
{
    const auto preempted = [&options]() {
        return options.preempt_requested && options.preempt_requested();
    };

    // 每次规划开始先清理上一轮局部结果。global_data_ 不在这里 reset，
    // 因为它是任务级参考轨迹，只由 planGlobalTraj() 维护。
    last_goal_ = goal;
    last_planned_target_ = goal;
    last_plan_reached_requested_goal_ = true;
    current_path_.clear();
    last_optimization_result_.clear();
    const double last_guide_astar_ms = last_timing_.guide_astar_ms;
    last_timing_.reset();
    last_timing_.guide_astar_ms = last_guide_astar_ms;
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
    // 对 GEN_NEW_TRAJ，FSM 会传入当前 odom 状态；对 REPLAN_TRAJ，FSM 会传入旧 MINCO 轨迹在未来切换时刻的
    // $p(t_s),v(t_s),a(t_s)$，这和 EGO-Planner 从当前执行轨迹上取重规划边界条件的原则一致。
    const Eigen::Vector3d start = options.has_start_state ? options.start_pos : currentPosition();
    if (preempted())
    {
        last_error_ = "Planning preempted before A*.";
        return false;
    }

    std::vector<Eigen::Vector3d> raw_path;
    astar_planner_->setCancelCallback(options.preempt_requested);
    const ros::WallTime astar_start = ros::WallTime::now();
    double astar_clearance_used = astar_config_.min_clearance;
    int astar_expanded_nodes = 0;

    // 前端 A* 负责给后端优化器提供拓扑路径。
    // 如果 planGlobalTraj() 刚刚用 guide A* 得到了 start -> local_target 的路径，
    // 且当前 planToGoal() 的起终点仍与该路径匹配，就复用这条 raw_path，避免同一段拓扑重复搜索。
    // 若复用校验失败，再走原来的 frontend map -> base map A* fallback。
    bool reused_guide_path = false;
    bool success = tryReuseCachedGuidePath(start,
                                           goal,
                                           options.preempt_requested,
                                           raw_path,
                                           astar_clearance_used,
                                           astar_expanded_nodes);
    if (success)
    {
        reused_guide_path = true;
    }
    else if (!preempted())
    {
        // 普通前端搜索会优先使用更保守的 frontend map，若失败再退回 base map；
        // 若起点落在 frontend 膨胀层内，会先执行 escape；若终点超出局部地图或搜索超时，
        // 可能返回投影点 / best-effort 终点。
        success = runAStarWithFallback(start,
                                       goal,
                                       options.preempt_requested,
                                       raw_path,
                                       astar_clearance_used,
                                       astar_expanded_nodes);
    }
    last_timing_.frontend_astar_ms = (ros::WallTime::now() - astar_start).toSec() * 1000.0;
    last_timing_.astar_nodes = astar_expanded_nodes;
    last_timing_.clearance_used = astar_clearance_used;
    astar_planner_->setCancelCallback(std::function<bool()>());
    if (reused_guide_path)
    {
        searched_nodes_cloud_ = centersToCloud(raw_path, ros::Time::now());
        ROS_INFO_THROTTLE(1.0,
                          "[FastNav][LocalPlannerManager] Reused cached guide path for backend reference. points=%zu",
                          raw_path.size());
    }
    else
    {
        searched_nodes_cloud_ = centersToCloud(astar_planner_->searchedNodes(), ros::Time::now());
    }

    if (preempted())
    {
        last_error_ = "Planning preempted after A*.";
        return false;
    }

    if (!success)
    {
        if (last_error_.empty())
        {
            last_error_ = astar_planner_->lastError();
        }
        return false;
    }

    if (!raw_path.empty())
    {
        last_planned_target_ = raw_path.back();
    }

    // 若 A* 只到达投影点或 best-effort 点，则 $p_{real} \ne p_g$。
    // FSM 会通过 lastPlanReachedRequestedGoal() 得知这一点，把 touch_goal 改成 false，后续继续滚动推进。
    last_plan_reached_requested_goal_ =
        (last_planned_target_ - goal).norm() < std::max(0.2, resolution_);

    size_t preserve_prefix_size = 0;
    const ros::WallTime reference_start = ros::WallTime::now();

    // 构造后端优化参考路径：
    // - 普通规划直接使用 raw_path；
    // - use_current_traj=true 时，前缀会复用旧 MINCO 剩余安全段；
    // - use_random_init=true 时，会对可变中间点加入扰动，改变 corridor / MINCO 初值。
    const std::vector<Eigen::Vector3d> optimization_reference_path =
        buildOptimizationReferencePath(raw_path, options, preserve_prefix_size);
    last_timing_.reference_ms = (ros::WallTime::now() - reference_start).toSec() * 1000.0;
    if (preempted())
    {
        last_error_ = "Planning preempted after reference path generation.";
        return false;
    }

    fastnav::MincoGcopterOptimizer::BoundaryState start_state;
    start_state.pos = start;
    if (options.has_start_state)
    {
        // FSM 已显式给出本次规划边界状态，通常来自 odom 或旧轨迹采样。
        start_state.vel = options.start_vel;
        start_state.acc = options.start_acc;
    }
    else
    {
        // 默认入口没有显式边界时，速度从 odom 读取，加速度先置零。
        start_state.vel = Eigen::Vector3d(current_odom_.twist.twist.linear.x,
                                          current_odom_.twist.twist.linear.y,
                                          current_odom_.twist.twist.linear.z);
        start_state.acc.setZero();
    }

    fastnav::MincoGcopterOptimizer::BoundaryState goal_state;
    goal_state.pos = last_planned_target_;
    goal_state.vel = options.goal_vel;
    goal_state.acc = options.goal_acc;
    const bool actual_touch_goal = options.touch_goal && last_plan_reached_requested_goal_;
    if (!last_plan_reached_requested_goal_)
    {
        // 当 A* 因局部地图边界或 TIME_OUT 只返回 $p_{best}$ 时，本次 MINCO 的终点应为
        // $p_{best}$，但任务级目标仍是原来的 $p_g$。这里给中间终点一个朝向 $p_g$ 的速度，
        // 即 $v_T = \hat d \min(v_{max}, \max(0.2,\|v_{ref}\|))$，避免每段 best-effort 都硬刹车。
        const Eigen::Vector3d to_requested_goal = goal - last_planned_target_;
        if (to_requested_goal.norm() > 1.0e-3)
        {
            const double ref_speed = options.goal_vel.norm() > 1.0e-3
                                         ? options.goal_vel.norm()
                                         : 0.5 * optimizer_config_.feasibility.max_vel;
            goal_state.vel = to_requested_goal.normalized() *
                             std::min(optimizer_config_.feasibility.max_vel,
                                      std::max(0.2, ref_speed));
        }
        else
        {
            goal_state.vel.setZero();
        }
        goal_state.acc.setZero();
    }

    // PathOptimizer 是后端核心：几何 shortcut -> 局部 surface 提取 -> safe corridor -> GCOPTER/MINCO -> fine check。
    // actual_touch_goal=false 时，fine check 只检查前段比例，和 EGO-v2 对非最终 local target 的宽松检查思想一致。
    if (path_optimizer_ && path_optimizer_->optimizeTrajectory(optimization_reference_path,
                                                               voxel_map_,
                                                               start_state,
                                                               goal_state,
                                                               last_optimization_result_,
                                                               preserve_prefix_size,
                                                               actual_touch_goal,
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
        last_timing_.shortcut_ms = last_optimization_result_.shortcut_ms;
        last_timing_.corridor_ms = last_optimization_result_.corridor_ms;
        last_timing_.minco_ms = last_optimization_result_.minco_ms;
        last_timing_.fine_check_ms = last_optimization_result_.fine_check_ms;
        last_timing_.corridor_num = static_cast<int>(last_optimization_result_.corridors.size());
        last_timing_.minco_retry_count = last_optimization_result_.minco_retry_count;
        if (!preempted())
        {
            recordOptimizationFailure(raw_path,
                                      optimization_reference_path,
                                      options,
                                      start,
                                      goal,
                                      actual_touch_goal);
        }
        return false;
    }

    if (current_path_.empty())
    {
        last_error_ = "Optimized path is empty.";
        return false;
    }

    const ros::Time time_now = ros::Time::now();
    const ros::Time traj_start_time = options.trajectory_start_time.isZero()
                                          ? time_now
                                          : options.trajectory_start_time;

    // raw_path / shortcut_path 用作几何回退，MINCO 成功时 local_data_ 保存连续可执行轨迹 $p(t)$。
    // global_data_ 是任务级轻量参考轨迹，只由 planGlobalTraj() 维护，不能被局部优化结果覆盖。
    // traj_start_time 可能是未来时刻 $t_{now}+\Delta t_f$，traj_server 会按这个时间 commit 新轨迹。
    updateTrajInfo(last_optimization_result_, traj_start_time);
    last_timing_.shortcut_ms = last_optimization_result_.shortcut_ms;
    last_timing_.corridor_ms = last_optimization_result_.corridor_ms;
    last_timing_.minco_ms = last_optimization_result_.minco_ms;
    last_timing_.fine_check_ms = last_optimization_result_.fine_check_ms;
    last_timing_.corridor_num = static_cast<int>(last_optimization_result_.corridors.size());
    last_timing_.minco_retry_count = last_optimization_result_.minco_retry_count;

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

// 复用 planGlobalTraj() 中 guide A* 的结果，避免同一段 start -> local_target 重复搜索。
// 复用不是无条件的：
// 1. 起点必须仍接近缓存路径首点，$\|p_s-p_{0}^{guide}\| < \epsilon_s$；
// 2. 目标必须仍接近缓存路径末点，$\|p_g-p_{N}^{guide}\| < \epsilon_g$；
// 3. 将首末点替换为当前 start / goal 后，每条线段都必须满足 base map 的 isLineFree()。
// 这样可以利用 guide A* 的拓扑结果，同时避免地图变化或重规划起点漂移导致后端拿到失效路径。
bool LocalPlannerManager::tryReuseCachedGuidePath(const Eigen::Vector3d& start,
                                                  const Eigen::Vector3d& goal,
                                                  const std::function<bool()>& preempt_requested,
                                                  std::vector<Eigen::Vector3d>& path,
                                                  double& clearance_used,
                                                  int& expanded_nodes)
{
    path.clear();
    clearance_used = cached_guide_clearance_used_;
    expanded_nodes = 0;

    if (!enable_guide_path_reuse_ || cached_guide_path_.size() < 2 || !voxel_map_)
    {
        return false;
    }

    if (preempt_requested && preempt_requested())
    {
        last_error_ = "Guide path reuse preempted.";
        return false;
    }

    const double start_error = (start - cached_guide_path_.front()).norm();
    const double goal_error = (goal - cached_guide_path_.back()).norm();
    if (start_error > guide_reuse_start_tolerance_ || goal_error > guide_reuse_goal_tolerance_)
    {
        return false;
    }

    std::vector<Eigen::Vector3d> candidate = cached_guide_path_;
    candidate.front() = start;
    candidate.back() = goal;

    const double step = std::max(0.5 * resolution_, astar_config_.line_check_step);
    for (size_t i = 0; i < candidate.size(); ++i)
    {
        if (preempt_requested && preempt_requested())
        {
            last_error_ = "Guide path reuse preempted during validation.";
            return false;
        }

        // out-of-map 在 VoxelMap::query() 中等价于 occupied。这里显式检查，
        // 是为了把“缓存路径已经滑出局部地图”的情况交回普通 A* 处理。
        if (!voxel_map_->isInMap(candidate[i]) || voxel_map_->isInflatedOccupied(candidate[i]))
        {
            return false;
        }

        if (i > 0 && !voxel_map_->isLineFree(candidate[i - 1], candidate[i], step))
        {
            return false;
        }
    }

    path.swap(candidate);
    return true;
}

// 前端几何路径搜索：从规划起点 $p_s$ 到当前 local target / goal $p_g$ 生成 raw_path。
// 这个函数和 runGuideAStarWithFallback() 的区别是：
// - runGuideAStarWithFallback() 只搜索到 horizon，用来选 global reference 的 local target；
// - runAStarWithFallback() 要尽量连到本次 planToGoal() 的真实目标，输出给 safe corridor / MINCO 后端。
//
// 搜索策略同样采用 SUPER 风格的分层 fallback：
// 1. 若起点落在保守 frontend inflated map 内，先用 base map 做 escape，找到 $p \notin \mathcal{O}_{front}$；
// 2. 优先在 frontend map 上搜索，得到更大 clearance 的路径；
// 3. frontend 失败后，如果 $min\_clearance>0$，退回 base map 再试一次；
// 4. 若 A* 内部发生 goal projection / best-effort，path.back() 可能不是原始 $p_g$，
//    planToGoal() 会通过 last_planned_target_ 把这个实际终点传给 MINCO。
bool LocalPlannerManager::runAStarWithFallback(const Eigen::Vector3d& start,
                                               const Eigen::Vector3d& goal,
                                               const std::function<bool()>& preempt_requested,
                                               std::vector<Eigen::Vector3d>& path,
                                               double& clearance_used,
                                               int& expanded_nodes)
{
    // 输出参数在入口统一重置，保证失败时不会沿用上一轮搜索结果。
    path.clear();
    clearance_used = astar_config_.min_clearance;
    expanded_nodes = 0;

    if (!astar_planner_ || !frontend_voxel_map_ || !voxel_map_)
    {
        last_error_ = "A* fallback dependencies are not ready.";
        return false;
    }

    // AStarPlanner 内部只持有一个 map 指针。函数退出前恢复 frontend map，
    // 避免下一次搜索误用 base map。
    auto restore_frontend_map = [this]() {
        astar_planner_->setMap(frontend_voxel_map_);
    };

    std::vector<Eigen::Vector3d> escape_prefix;
    Eigen::Vector3d search_start = start;
    if (astar_config_.enable_escape_search && frontend_voxel_map_->isInMap(start) &&
        frontend_voxel_map_->isInflatedOccupied(start))
    {
        // Escape search 的目标不是原 goal，而是先从保守前端膨胀集合 $\mathcal{O}_{front}$ 中出去。
        // 它使用 base map 作为通行约束，寻找第一个满足 $p \notin \mathcal{O}_{front}$ 的出口点。
        // 这样当起点贴着障碍、被额外 clearance 包住时，A* 仍然能先找到一小段可执行前缀。
        astar_planner_->setMap(voxel_map_);
        if (astar_planner_->escapeFromInflatedRegion(start, frontend_voxel_map_, escape_prefix) &&
            escape_prefix.size() >= 2)
        {
            search_start = escape_prefix.back();
            expanded_nodes += astar_planner_->lastExpandedNodes();
            ROS_WARN_THROTTLE(1.0,
                              "[FastNav][LocalPlannerManager] Escape prefix generated before frontend A*: %.2fm.",
                              (search_start - start).norm());
        }
        else if (astar_planner_->lastStatus() == AStarPlanner::SearchStatus::PREEMPTED ||
                 (preempt_requested && preempt_requested()))
        {
            // 若新目标抢占，当前旧目标搜索立刻终止，避免旧轨迹稍后覆盖新目标。
            last_error_ = astar_planner_->lastError();
            restore_frontend_map();
            return false;
        }
    }

    // 第一层：frontend map。其膨胀半径为 $r_{front}=r_{base}+min\_clearance$，
    // 能让前端路径尽量偏向宽通道中心，为后端 FIRI corridor 和 MINCO 留更大可行域。
    astar_planner_->setMap(frontend_voxel_map_);
    if (astar_planner_->plan(search_start, goal, path))
    {
        expanded_nodes += astar_planner_->lastExpandedNodes();
        clearance_used = astar_config_.min_clearance;
        prependPathPrefix(escape_prefix, path);
        restore_frontend_map();
        return true;
    }

    const std::string frontend_error = astar_planner_->lastError();
    const AStarPlanner::SearchStatus frontend_status = astar_planner_->lastStatus();
    expanded_nodes += astar_planner_->lastExpandedNodes();

    if (preempt_requested && preempt_requested())
    {
        last_error_ = frontend_error;
        restore_frontend_map();
        return false;
    }

    if (astar_config_.min_clearance <= 1.0e-6)
    {
        // $min\_clearance=0$ 时 frontend map 已经等价于 base map，没有必要重复搜索。
        last_error_ = frontend_error;
        restore_frontend_map();
        return false;
    }

    ROS_WARN_THROTTLE(1.0,
                      "[FastNav][LocalPlannerManager] Frontend A* failed, retry once on base map: %s",
                      frontend_error.c_str());

    // 第二层：base map fallback。它只包含无人机半径和 safety margin 的基础膨胀，
    // 比 frontend map 更宽松；代价是 clearance_used=0，后端 fine check 仍会用 base map 保底。
    astar_planner_->setMap(voxel_map_);
    if (astar_planner_->plan(search_start, goal, path))
    {
        expanded_nodes += astar_planner_->lastExpandedNodes();
        clearance_used = 0.0;
        prependPathPrefix(escape_prefix, path);
        ROS_WARN_THROTTLE(1.0,
                          "[FastNav][LocalPlannerManager] Base-map A* fallback succeeded; path has lower frontend clearance.");
        restore_frontend_map();
        return true;
    }

    const std::string base_error = astar_planner_->lastError();
    const AStarPlanner::SearchStatus base_status = astar_planner_->lastStatus();
    expanded_nodes += astar_planner_->lastExpandedNodes();
    clearance_used = 0.0;
    last_error_ = "Frontend A* failed: " + frontend_error +
                  " [" + searchStatusName(frontend_status) + "]" +
                  "; base-map fallback failed: " + base_error +
                  " [" + searchStatusName(base_status) + "]";
    restore_frontend_map();
    return false;
}

// guide search 只负责给全局参考轨迹选择“下一段局部目标”，不负责生成最终可执行轨迹。
// 输入是当前规划起点 $p_s$、任务最终目标 $p_g$ 和规划视距 $H$；输出 path 的末端就是 local target。
// 搜索策略仿照 SUPER 的分层前端：先使用更保守的 frontend inflated map，失败后退回 base inflated map。
// 若起点落在 frontend 膨胀区内，会先执行 escape search，寻找 $p \notin \mathcal{O}_{front}$ 的出口点。
// 若 A* 超时，则尝试缩短视距 $H'=\max(H_{min},\alpha H)$，优先给后端一段可推进的短路径。
bool LocalPlannerManager::runGuideAStarWithFallback(const Eigen::Vector3d& start,
                                                    const Eigen::Vector3d& final_goal,
                                                    double horizon,
                                                    std::vector<Eigen::Vector3d>& path,
                                                    double& clearance_used,
                                                    int& expanded_nodes)
{
    // 输出参数在函数入口统一清零，保证失败时上层不会误用上一次搜索残留数据。
    path.clear();
    clearance_used = astar_config_.min_clearance;
    expanded_nodes = 0;

    if (!astar_planner_ || !frontend_voxel_map_ || !voxel_map_)
    {
        last_error_ = "Guide A* fallback dependencies are not ready.";
        return false;
    }

    // AStarPlanner 内部只持有一个 map 指针。函数返回前恢复 frontend map，避免后续调用继承 base map。
    auto restore_frontend_map = [this]() {
        astar_planner_->setMap(frontend_voxel_map_);
    };

    std::vector<Eigen::Vector3d> escape_prefix;
    Eigen::Vector3d search_start = start;
    if (astar_config_.enable_escape_search && frontend_voxel_map_->isInMap(start) &&
        frontend_voxel_map_->isInflatedOccupied(start))
    {
        // 起点可能因为较大的 frontend clearance 落入 $\mathcal{O}_{front}$，但在 base map 中仍安全。
        // 此时直接在 frontend map 上 A* 会没有可扩展邻居，因此先在 base map 中搜索一小段 escape_prefix。
        astar_planner_->setMap(voxel_map_);
        if (astar_planner_->escapeFromInflatedRegion(start, frontend_voxel_map_, escape_prefix) &&
            escape_prefix.size() >= 2)
        {
            // 后续 guide A* 从 escape 末端继续搜索；最终返回路径会重新拼回 escape_prefix。
            search_start = escape_prefix.back();
            expanded_nodes += astar_planner_->lastExpandedNodes();
        }
        else if (astar_planner_->lastStatus() == AStarPlanner::SearchStatus::PREEMPTED)
        {
            last_error_ = astar_planner_->lastError();
            restore_frontend_map();
            return false;
        }
    }

    auto try_horizon_search = [this,
                               &search_start,
                               &final_goal,
                               &path,
                               &expanded_nodes,
                               &escape_prefix](const std::shared_ptr<fastnav_mapping::VoxelMap>& map,
                                                double query_horizon) {
        // planToHorizon() 的停止条件不是必须到达 $p_g$，而是满足累计路径长度 $g \ge H$。
        // 因此 local target 是“沿可通行拓扑向最终目标推进 horizon 距离”的节点。
        astar_planner_->setMap(map);
        if (astar_planner_->planToHorizon(search_start, final_goal, query_horizon, path))
        {
            expanded_nodes += astar_planner_->lastExpandedNodes();
            prependPathPrefix(escape_prefix, path);
            return true;
        }

        expanded_nodes += astar_planner_->lastExpandedNodes();
        if (astar_planner_->lastStatus() == AStarPlanner::SearchStatus::TIME_OUT)
        {
            // TIME_OUT 降级：若 $H$ 内搜索太慢，缩短为 $H'=\max(H_{min},\alpha H)$。
            // 这和 SUPER 的 REACH_HORIZON 思想一致：先给后端一个较短、可推进的 local target，
            // 不让前端为了完整 horizon 长时间阻塞控制闭环。
            const double shorter_horizon =
                std::max(astar_config_.timeout_min_horizon,
                         astar_config_.timeout_horizon_scale * query_horizon);
            if (shorter_horizon + 1.0e-6 < query_horizon)
            {
                if (astar_planner_->planToHorizon(search_start, final_goal, shorter_horizon, path))
                {
                    expanded_nodes += astar_planner_->lastExpandedNodes();
                    prependPathPrefix(escape_prefix, path);
                    return true;
                }
                expanded_nodes += astar_planner_->lastExpandedNodes();
            }
        }
        return false;
    };

    // 第一层：保守 frontend map。其膨胀半径为 $r_{front}=r_{base}+min\_clearance$，
    // 选出的 local target 更偏向宽通道中心，能给后端 corridor / MINCO 留更大优化空间。
    astar_planner_->setMap(frontend_voxel_map_);
    if (try_horizon_search(frontend_voxel_map_, horizon))
    {
        clearance_used = astar_config_.min_clearance;
        restore_frontend_map();
        return true;
    }

    const std::string frontend_error = astar_planner_->lastError();
    const AStarPlanner::SearchStatus frontend_status = astar_planner_->lastStatus();

    if (astar_config_.min_clearance <= 1.0e-6)
    {
        last_error_ = frontend_error;
        restore_frontend_map();
        return false;
    }

    ROS_WARN_THROTTLE(1.0,
                      "[FastNav][LocalPlannerManager] Guide A* failed on frontend map, retry once on base map: %s",
                      frontend_error.c_str());

    // 第二层：base map fallback。它只包含无人机半径和安全距离的基础膨胀，
    // 通行性更宽松，等价于从 SUPER 的 inflated map 退回更原始的可通行概率地图思想。
    astar_planner_->setMap(voxel_map_);
    if (try_horizon_search(voxel_map_, horizon))
    {
        clearance_used = 0.0;
        ROS_WARN_THROTTLE(1.0,
                          "[FastNav][LocalPlannerManager] Guide A* base-map fallback succeeded.");
        restore_frontend_map();
        return true;
    }

    const std::string base_error = astar_planner_->lastError();
    const AStarPlanner::SearchStatus base_status = astar_planner_->lastStatus();
    clearance_used = 0.0;
    last_error_ = "Guide frontend A* failed: " + frontend_error +
                  " [" + searchStatusName(frontend_status) + "]" +
                  "; guide base-map fallback failed: " + base_error +
                  " [" + searchStatusName(base_status) + "]";
    restore_frontend_map();
    return false;
}

// 将 A* / guide A* 得到的 raw_path 整理成后端优化参考路径。
// 这个函数是前端搜索和后端 safe corridor / MINCO 之间的接口层：
// 1. 普通规划时，参考路径就是 A* 离散路径 $\{p_i\}_{i=0}^{N}$；
// 2. 执行中重规划时，可把旧 MINCO 剩余安全段拼到 raw_path 前面，得到
//    $p_{ref}=[p_{old}(t_s),...,p_{old}(t_b),p_{astar,k},...,p_g]$；
// 3. 连续失败后的随机尝试会扰动一个可变中间点，让 FIRI 走廊和 MINCO 初值改变，
//    避免每次都卡在完全相同的 corridor / 局部最优里。
std::vector<Eigen::Vector3d> LocalPlannerManager::buildOptimizationReferencePath(
    const std::vector<Eigen::Vector3d>& raw_path,
    const ReplanOptions& options,
    size_t& preserve_prefix_size) const
{
    // preserve_prefix_size 会传给 PathOptimizer::shortcutPath()。
    // 若前缀来自旧轨迹，shortcut 不能删除这段点列，否则新轨迹无法在 $p,v,a$ 上平滑接续旧轨迹。
    preserve_prefix_size = 0;

    // use_current_traj=true 对应 EGO 风格的“从当前执行轨迹上取重规划起点，并保留一段旧轨迹前缀”。
    // 否则直接使用 A* 输出的 raw_path 作为后端参考。
    std::vector<Eigen::Vector3d> reference_path =
        options.use_current_traj ? buildCurrentTrajReferencePath(raw_path, options, preserve_prefix_size) : raw_path;

    // 随机初始化只在 retry 阶段启用。若 reference_path 不足两个点、地图不可用、或随机半径为 0，
    // 直接返回原参考路径，让后端按确定性 shortcut/corridor/MINCO 流程执行。
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

    // 选择一个“可变中间点”作为扰动对象。
    // 若路径已有足够多点，就取可变段中部的点 $r_m$；否则在 start-goal 中间插入一个新点。
    // prev / next 是扰动点前后的锚点，后续候选点必须满足 $lineFree(prev,candidate)$ 和 $lineFree(candidate,next)$。
    const bool has_middle_point = reference_path.size() > mutable_begin + 2;
    const size_t mid_id = has_middle_point ? mutable_begin + (reference_path.size() - mutable_begin) / 2 : mutable_begin;
    const Eigen::Vector3d prev = has_middle_point ? reference_path[mid_id - 1] : start;
    const Eigen::Vector3d next = has_middle_point ? reference_path[mid_id + 1] : goal;
    const Eigen::Vector3d center = has_middle_point ? reference_path[mid_id] : 0.5 * (start + goal);

    // 在局部段方向 $d=(next-prev)/\|next-prev\|$ 的法平面上构造两个基向量 $e_1,e_2$，
    // 随机候选点使用 $p_c = center + r(\cos\theta e_1 + \sin\theta e_2)$。
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

    // 连续失败次数越多，扰动半径越大：$r = s(1+0.25 n_{fail})\|p_g-p_s\|$，
    // 并限制在地图分辨率和 3m 之间，避免扰动过小无意义或过大跳出局部地图。
    const double failure_gain = 1.0 + 0.25 * std::max(0, options.continuous_failures);
    const double base_radius = std::min(3.0, std::max(voxel_map_->resolution(),
                                                      random_init_scale_ * failure_gain * path_span * 0.15));
    const int seed = std::max(0, options.attempt) + std::max(0, options.continuous_failures);

    // 最多尝试 8 个候选点，角度每次转 $45^\circ$，半径逐步放大。
    // 只有候选点自身 free，且前后两条连接线均无碰撞，才会写回 reference_path。
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
            // 路径已有中间点时，直接替换这个点，保持路径点数量不变。
            reference_path[mid_id] = candidate;
        }
        else
        {
            // 路径只有起终点时，插入一个中间点，让 corridor 至少由两段局部线段支撑。
            const size_t insert_id = preserve_prefix_size > 0 ? mutable_begin : 1;
            reference_path.insert(reference_path.begin() + static_cast<std::ptrdiff_t>(insert_id), candidate);
        }
        return reference_path;
    }

    // 所有随机候选都不可行时，保留原 reference_path。
    // 这比强行写入一个可能碰撞的点更安全，后端仍可按原始 A* 路径尝试一次。
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
                                         const ros::Time& start_time)
{
    local_data_.reset();
    if (!result.has_minco || !result.minco_traj.valid())
    {
        return;
    }

    // LocalTrajData 保存当前可执行 MINCO 轨迹 $p(t)$，并额外缓存采样点用于 RViz / path topic。
    local_data_.setLocalTraj(result.minco_traj.trajectory(),
                             start_time,
                             frame_id_,
                             minco_sample_dt_);
    local_data_.corridor_ = result.corridors;
    if (!result.sampled_path.empty())
    {
        local_data_.sampled_path_ = result.sampled_path;
    }
}

void LocalPlannerManager::recordOptimizationFailure(const std::vector<Eigen::Vector3d>& frontend_path,
                                                    const std::vector<Eigen::Vector3d>& reference_path,
                                                    const ReplanOptions& options,
                                                    const Eigen::Vector3d& start,
                                                    const Eigen::Vector3d& requested_goal,
                                                    bool touch_goal)
{
    if (!debug_recorder_.enabled())
    {
        return;
    }

    PlannerDebugRecorder::Snapshot snapshot;
    snapshot.stamp = ros::Time::now();
    snapshot.frame_id = frame_id_;
    snapshot.state = options.use_current_traj ? "REPLAN_TRAJ" : "GEN_NEW_TRAJ";
    if (options.use_random_init)
    {
        snapshot.state += "_RANDOM";
    }
    snapshot.reason = last_error_;
    snapshot.attempt = options.attempt;
    snapshot.continuous_failures = options.continuous_failures;
    snapshot.use_current_traj = options.use_current_traj;
    snapshot.use_random_init = options.use_random_init;
    snapshot.touch_goal = touch_goal;
    snapshot.reached_requested_goal = last_plan_reached_requested_goal_;
    snapshot.odom_pos = currentPosition();
    snapshot.start = start;
    snapshot.requested_goal = requested_goal;
    snapshot.planned_target = last_planned_target_;

    snapshot.timing.guide_astar_ms = last_timing_.guide_astar_ms;
    snapshot.timing.frontend_astar_ms = last_timing_.frontend_astar_ms;
    snapshot.timing.reference_ms = last_timing_.reference_ms;
    snapshot.timing.shortcut_ms = last_timing_.shortcut_ms;
    snapshot.timing.corridor_ms = last_timing_.corridor_ms;
    snapshot.timing.minco_ms = last_timing_.minco_ms;
    snapshot.timing.fine_check_ms = last_timing_.fine_check_ms;
    snapshot.timing.astar_nodes = last_timing_.astar_nodes;
    snapshot.timing.corridor_num = last_timing_.corridor_num;
    snapshot.timing.minco_retry_count = last_timing_.minco_retry_count;
    snapshot.timing.clearance_used = last_timing_.clearance_used;

    snapshot.frontend_path = frontend_path;
    snapshot.reference_path = reference_path;
    snapshot.shortcut_path = last_optimization_result_.shortcut_path;
    snapshot.sampled_path = last_optimization_result_.sampled_path;
    snapshot.corridors = last_optimization_result_.corridors;
    if (astar_planner_)
    {
        snapshot.searched_nodes = astar_planner_->searchedNodes();
    }

    snapshot.has_minco = last_optimization_result_.has_minco && last_optimization_result_.minco_traj.valid();
    snapshot.minco_traj = last_optimization_result_.minco_traj;

    snapshot.has_filtered_cloud = !latest_filtered_cloud_.data.empty();
    snapshot.has_occupied_cloud = !debug_occupied_cloud_.data.empty();
    snapshot.has_inflated_cloud = !debug_inflated_cloud_.data.empty();
    if (snapshot.has_filtered_cloud)
    {
        snapshot.filtered_cloud = latest_filtered_cloud_;
    }
    if (snapshot.has_occupied_cloud)
    {
        snapshot.occupied_cloud = debug_occupied_cloud_;
    }
    if (snapshot.has_inflated_cloud)
    {
        snapshot.inflated_cloud = debug_inflated_cloud_;
    }

    debug_recorder_.recordMincoFailure(snapshot);
}

// 从 current_odom_ 中提取当前位置向量，作为局部地图中心和规划起点。
Eigen::Vector3d LocalPlannerManager::currentPosition() const
{
    return Eigen::Vector3d(current_odom_.pose.pose.position.x,
                           current_odom_.pose.pose.position.y,
                           current_odom_.pose.pose.position.z);
}

}  // namespace fastnav_planner
