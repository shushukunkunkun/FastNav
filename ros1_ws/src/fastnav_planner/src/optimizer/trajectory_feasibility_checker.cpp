#include "fastnav_planner/optimizer/trajectory_feasibility_checker.h"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace fastnav_planner
{

void TrajectoryFeasibilityChecker::Result::reset()
{
    feasible = false;
    violation_type.clear();
    message.clear();
    violation_time = 0.0;
    position.setZero();
    value.setZero();
    max_vel = 0.0;
    max_acc = 0.0;
    max_jerk = 0.0;
}

void TrajectoryFeasibilityChecker::setConfig(const Config& config)
{
    config_ = config;
    config_.sample_dt = std::max(1.0e-3, config_.sample_dt);
    config_.collision_check_step = std::max(1.0e-3, config_.collision_check_step);
    config_.check_horizon_ratio = std::min(1.0, std::max(0.05, config_.check_horizon_ratio));
    config_.tolerance = std::max(0.0, config_.tolerance);
}

bool TrajectoryFeasibilityChecker::check(const fastnav::MincoTraj& traj,
                                         const std::shared_ptr<fastnav_mapping::VoxelMap>& map,
                                         Result& result,
                                         bool touch_goal) const
{
    result.reset();
    if (!traj.valid())
    {
        result.violation_type = "invalid_traj";
        result.message = "MINCO trajectory is invalid.";
        return false;
    }

    if (config_.check_collision && !checkCollision(traj, map, result, touch_goal))
    {
        result.message = formatViolation(result);
        return false;
    }

    if (config_.check_dynamics && !checkDynamics(traj, result))
    {
        result.message = formatViolation(result);
        return false;
    }

    result.feasible = true;
    result.message = "Trajectory is feasible.";
    return true;
}

bool TrajectoryFeasibilityChecker::checkCollision(
    const fastnav::MincoTraj& traj,
    const std::shared_ptr<fastnav_mapping::VoxelMap>& map,
    Result& result,
    bool touch_goal) const
{
    if (!map)
    {
        result.violation_type = "no_map";
        return false;
    }

    const double duration = traj.getDuration();
    // EGO-v2 的局部轨迹 fine check 只检查靠近执行端的前段；如果当前 local target 已经是最终目标，
    // 则必须检查完整轨迹。这里用 $T_c=T$ 或 $T_c=\rho T$ 表示实际碰撞检查时长。
    const double horizon = touch_goal ? duration : duration * config_.check_horizon_ratio;
    const int sample_num = std::max(1, static_cast<int>(std::ceil(horizon / config_.sample_dt)));

    Eigen::Vector3d last_pos = traj.getPosition(0.0);
    for (int i = 0; i <= sample_num; ++i)
    {
        const double t = std::min(horizon, static_cast<double>(i) * config_.sample_dt);
        const Eigen::Vector3d pos = traj.getPosition(t);

        if (!map->isInMap(pos))
        {
            result.violation_type = "out_of_map";
            result.violation_time = t;
            result.position = pos;
            return false;
        }

        if (map->isInflatedOccupied(pos))
        {
            result.violation_type = "collision";
            result.violation_time = t;
            result.position = pos;
            return false;
        }

        if (i > 0 && !map->isLineFree(last_pos, pos, config_.collision_check_step))
        {
            result.violation_type = "line_collision";
            result.violation_time = t;
            result.position = pos;
            return false;
        }

        last_pos = pos;
    }

    return true;
}

bool TrajectoryFeasibilityChecker::checkDynamics(const fastnav::MincoTraj& traj,
                                                 Result& result) const
{
    fastnav::MincoTraj::DynamicFeasibilityResult dynamic_result;
    if (!traj.checkDynamicFeasibility(config_.max_vel,
                                      config_.max_acc,
                                      config_.max_jerk,
                                      config_.tolerance,
                                      config_.sample_dt,
                                      &dynamic_result))
    {
        result.violation_type = dynamic_result.violation_type;
        result.violation_time = dynamic_result.first_violation_time;
        result.position = traj.getPosition(result.violation_time);
        result.value = dynamic_result.value;
        result.max_vel = dynamic_result.max_vel;
        result.max_acc = dynamic_result.max_acc;
        result.max_jerk = dynamic_result.max_jerk;
        return false;
    }

    result.max_vel = dynamic_result.max_vel;
    result.max_acc = dynamic_result.max_acc;
    result.max_jerk = dynamic_result.max_jerk;
    return true;
}

std::string TrajectoryFeasibilityChecker::formatViolation(const Result& result) const
{
    std::ostringstream oss;
    oss << "violation=" << result.violation_type
        << ", t=" << result.violation_time
        << ", p=[" << result.position.x() << ", "
        << result.position.y() << ", "
        << result.position.z() << "]";
    if (!result.value.isZero(1.0e-9))
    {
        oss << ", value=[" << result.value.x() << ", "
            << result.value.y() << ", "
            << result.value.z() << "]";
    }
    if (result.max_vel > 0.0 || result.max_acc > 0.0 || result.max_jerk > 0.0)
    {
        oss << ", max(v,a,j)=[" << result.max_vel << ", "
            << result.max_acc << ", "
            << result.max_jerk << "]";
    }
    return oss.str();
}

}  // namespace fastnav_planner
