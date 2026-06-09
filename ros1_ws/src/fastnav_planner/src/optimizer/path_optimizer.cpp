#include "fastnav_planner/optimizer/path_optimizer.h"

#include <algorithm>
#include <cmath>

namespace fastnav_planner
{

void PathOptimizer::setConfig(const Config& config)
{
    config_ = config;
    config_.line_check_step = std::max(1.0e-3, config_.line_check_step);
    config_.minco_sample_dt = std::max(1.0e-3, config_.minco_sample_dt);
    config_.max_retry = std::max(1, config_.max_retry);
    config_.corridor_range_retry_scale = std::max(0.1, config_.corridor_range_retry_scale);
    config_.corridor_progress_retry_scale = std::max(0.1, config_.corridor_progress_retry_scale);
    config_.weight_time_retry_scale = std::max(0.1, config_.weight_time_retry_scale);
    config_.penalty_pos_retry_scale = std::max(0.1, config_.penalty_pos_retry_scale);
    config_.penalty_vel_retry_scale = std::max(0.1, config_.penalty_vel_retry_scale);
    config_.penalty_body_rate_retry_scale = std::max(0.1, config_.penalty_body_rate_retry_scale);
    config_.penalty_tilt_retry_scale = std::max(0.1, config_.penalty_tilt_retry_scale);
    config_.penalty_thrust_retry_scale = std::max(0.1, config_.penalty_thrust_retry_scale);
    corridor_generator_.setConfig(config_.corridor);
    minco_optimizer_.setConfig(config_.minco);
    feasibility_checker_.setConfig(config_.feasibility);
}

bool PathOptimizer::optimizeTrajectory(const std::vector<Eigen::Vector3d>& raw_path,
                                       const std::shared_ptr<fastnav_mapping::VoxelMap>& map,
                                       const fastnav::MincoGcopterOptimizer::BoundaryState& start_state,
                                       const fastnav::MincoGcopterOptimizer::BoundaryState& goal_state,
                                       OptimizationResult& result,
                                       size_t preserve_prefix_size,
                                       bool touch_goal,
                                       const std::function<bool()>& preempt_requested)
{
    last_error_.clear();
    result.clear();
    result.raw_path = raw_path;

    if (raw_path.empty())
    {
        last_error_ = "Raw path is empty.";
        return false;
    }

    if (!config_.enable)
    {
        result.shortcut_path = raw_path;
        result.sampled_path = raw_path;
        return true;
    }

    if (!map)
    {
        last_error_ = "VoxelMap is not set.";
        return false;
    }

    if (preempt_requested && preempt_requested())
    {
        last_error_ = "Path optimization preempted before shortcut.";
        return false;
    }

    if (config_.shortcut)
    {
        if (!shortcutPath(raw_path, map, result.shortcut_path, preserve_prefix_size))
        {
            return false;
        }
    }
    else
    {
        result.shortcut_path = raw_path;
    }
    result.sampled_path = result.shortcut_path;

    if (!config_.minco_enable)
    {
        return true;
    }

    if (result.shortcut_path.size() < 2)
    {
        last_error_ = "Shortcut path needs at least two points for MINCO.";
        return !config_.require_minco;
    }

    fastnav::MincoGcopterOptimizer::BoundaryState minco_start = start_state;
    fastnav::MincoGcopterOptimizer::BoundaryState minco_goal = goal_state;
    minco_start.pos = result.shortcut_path.front();
    minco_goal.pos = result.shortcut_path.back();
    minco_optimizer_.setCancelCallback(preempt_requested);

    for (int attempt = 0; attempt < config_.max_retry; ++attempt)
    {
        if (preempt_requested && preempt_requested())
        {
            last_error_ = "Path optimization preempted before retry.";
            minco_optimizer_.setCancelCallback(std::function<bool()>());
            return false;
        }

        const double corridor_range_scale = std::pow(config_.corridor_range_retry_scale, attempt);
        const double corridor_progress_scale = std::pow(config_.corridor_progress_retry_scale, attempt);
        const double weight_time_scale = std::pow(config_.weight_time_retry_scale, attempt);
        const double penalty_pos_scale = std::pow(config_.penalty_pos_retry_scale, attempt);
        const double penalty_vel_scale = std::pow(config_.penalty_vel_retry_scale, attempt);
        const double penalty_body_rate_scale = std::pow(config_.penalty_body_rate_retry_scale, attempt);
        const double penalty_tilt_scale = std::pow(config_.penalty_tilt_retry_scale, attempt);
        const double penalty_thrust_scale = std::pow(config_.penalty_thrust_retry_scale, attempt);

        traj_utils::SafeCorridorGenerator::Config attempt_corridor_config = config_.corridor;
        attempt_corridor_config.range *= corridor_range_scale;
        attempt_corridor_config.progress *= corridor_progress_scale;
        corridor_generator_.setConfig(attempt_corridor_config);

        std::vector<Eigen::MatrixX4d> attempt_corridors;
        if (!generateCorridor(result.shortcut_path, map, attempt_corridors))
        {
            last_error_ = "Attempt " + std::to_string(attempt + 1) +
                          " corridor failed: " + last_error_;
            continue;
        }
        if (preempt_requested && preempt_requested())
        {
            last_error_ = "Path optimization preempted after corridor generation.";
            minco_optimizer_.setCancelCallback(std::function<bool()>());
            return false;
        }

        fastnav::MincoGcopterOptimizer::Config attempt_minco_config = config_.minco;
        attempt_minco_config.weight_time *= weight_time_scale;
        attempt_minco_config.penalty_pos *= penalty_pos_scale;
        attempt_minco_config.penalty_vel *= penalty_vel_scale;
        attempt_minco_config.penalty_body_rate *= penalty_body_rate_scale;
        attempt_minco_config.penalty_tilt *= penalty_tilt_scale;
        attempt_minco_config.penalty_thrust *= penalty_thrust_scale;
        minco_optimizer_.setConfig(attempt_minco_config);

        fastnav::MincoTraj attempt_traj;
        if (!minco_optimizer_.optimize(minco_start, minco_goal, attempt_corridors, attempt_traj))
        {
            last_error_ = "Attempt " + std::to_string(attempt + 1) +
                          " MINCO failed: " + minco_optimizer_.lastError();
            if (preempt_requested && preempt_requested())
            {
                minco_optimizer_.setCancelCallback(std::function<bool()>());
                return false;
            }
            continue;
        }
        if (preempt_requested && preempt_requested())
        {
            last_error_ = "Path optimization preempted after MINCO.";
            minco_optimizer_.setCancelCallback(std::function<bool()>());
            return false;
        }

        TrajectoryFeasibilityChecker::Result feasibility;
        if (!feasibility_checker_.check(attempt_traj, map, feasibility, touch_goal))
        {
            last_error_ = "Attempt " + std::to_string(attempt + 1) +
                          " fine check failed: " + feasibility.message;
            continue;
        }

        result.corridors = attempt_corridors;
        result.minco_traj = attempt_traj;
        result.has_minco = true;
        result.sampled_path = result.minco_traj.samplePositions(config_.minco_sample_dt);
        if (result.sampled_path.empty())
        {
            result.has_minco = false;
            last_error_ = "Attempt " + std::to_string(attempt + 1) +
                          " MINCO trajectory sampling returned empty path.";
            continue;
        }

        last_error_.clear();
        minco_optimizer_.setCancelCallback(std::function<bool()>());
        return true;
    }

    if (!config_.require_minco)
    {
        result.sampled_path = result.shortcut_path;
        minco_optimizer_.setCancelCallback(std::function<bool()>());
        return true;
    }

    minco_optimizer_.setCancelCallback(std::function<bool()>());
    return false;
}

bool PathOptimizer::optimize(const std::vector<Eigen::Vector3d>& raw_path,
                             const std::shared_ptr<fastnav_mapping::VoxelMap>& map,
                             std::vector<Eigen::Vector3d>& optimized_path)
{
    optimized_path.clear();

    fastnav::MincoGcopterOptimizer::BoundaryState start;
    fastnav::MincoGcopterOptimizer::BoundaryState goal;
    if (!raw_path.empty())
    {
        start.pos = raw_path.front();
        goal.pos = raw_path.back();
    }

    OptimizationResult result;
    const bool success = optimizeTrajectory(raw_path, map, start, goal, result);
    if (!success)
    {
        return false;
    }

    optimized_path = result.sampled_path.empty() ? result.shortcut_path : result.sampled_path;
    return !optimized_path.empty();
}

std::string PathOptimizer::lastError() const
{
    return last_error_;
}

bool PathOptimizer::shortcutPath(const std::vector<Eigen::Vector3d>& raw_path,
                                 const std::shared_ptr<fastnav_mapping::VoxelMap>& map,
                                 std::vector<Eigen::Vector3d>& optimized_path,
                                 size_t preserve_prefix_size) const
{
    if (raw_path.size() <= 2)
    {
        optimized_path = raw_path;
        return true;
    }

    // 从当前点 $p_i$ 开始，寻找最远的 $p_j$，使线段 $p_i -> p_j$ 不穿过膨胀障碍。
    // 这样可以保留路径拓扑，同时删除 A* 栅格搜索带来的锯齿点。
    // 当 preserve_prefix_size > 1 时，前缀点来自当前旧 MINCO 轨迹的剩余安全段，必须原样保留；
    // shortcut 只从前缀末端开始处理后续 A* 桥接段，避免破坏新旧轨迹切换的连续性。
    const size_t preserve_num = std::min(preserve_prefix_size, raw_path.size());
    size_t i = 0;
    if (preserve_num > 1)
    {
        optimized_path.insert(optimized_path.end(), raw_path.begin(), raw_path.begin() + preserve_num);
        i = preserve_num - 1;
    }
    else
    {
        optimized_path.push_back(raw_path.front());
    }

    while (i + 1 < raw_path.size())
    {
        size_t best = i + 1;
        for (size_t j = raw_path.size() - 1; j > i + 1; --j)
        {
            if (map->isLineFree(raw_path[i], raw_path[j], config_.line_check_step))
            {
                best = j;
                break;
            }
        }

        optimized_path.push_back(raw_path[best]);
        i = best;
    }

    return true;
}

bool PathOptimizer::generateCorridor(const std::vector<Eigen::Vector3d>& path,
                                     const std::shared_ptr<fastnav_mapping::VoxelMap>& map,
                                     std::vector<Eigen::MatrixX4d>& corridors)
{
    corridors.clear();
    if (!map)
    {
        last_error_ = "VoxelMap is not set for corridor generation.";
        return false;
    }

    std::vector<Eigen::Vector3d> obstacle_surface_points;
    // getSurf() 提取 occupied / inflated 表面体素中心，作为 FIRI 的障碍点集 $p_j$。
    // 只使用表面点可以减少约束规模，同时保持走廊边界由最近障碍决定。
    map->getSurf(obstacle_surface_points, true);

    if (!corridor_generator_.generate(path,
                                      obstacle_surface_points,
                                      map->getOrigin(),
                                      map->getCorner(),
                                      corridors))
    {
        last_error_ = "Corridor generation failed: " + corridor_generator_.lastError();
        return false;
    }

    if (corridors.empty())
    {
        last_error_ = "Corridor generation returned empty corridor.";
        return false;
    }

    return true;
}

}  // namespace fastnav_planner
