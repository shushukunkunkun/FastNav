#include "traj_utils/minco/minco_gcopter_optimizer.h"

#include <algorithm>
#include <cmath>

#include <traj_utils/minco/gcopter_optimizer.h>

namespace fastnav
{

void MincoGcopterOptimizer::setConfig(const Config& config)
{
    config_ = config;
    config_.weight_time = std::max(0.0, config_.weight_time);
    config_.smoothing_eps = std::max(1.0e-6, config_.smoothing_eps);
    config_.integral_intervals = std::max(1, config_.integral_intervals);
    config_.rel_cost_tol = std::max(1.0e-9, config_.rel_cost_tol);
    config_.length_per_piece = std::max(0.1, config_.length_per_piece);
    config_.max_vel = std::max(0.1, config_.max_vel);
    config_.max_body_rate = std::max(0.1, config_.max_body_rate);
    config_.max_tilt_angle = std::max(0.01, config_.max_tilt_angle);
    config_.max_thrust = std::max(config_.min_thrust + 1.0e-3, config_.max_thrust);
    config_.mass = std::max(0.1, config_.mass);
    config_.gravity = std::max(0.1, config_.gravity);
    config_.speed_eps = std::max(1.0e-6, config_.speed_eps);
}

void MincoGcopterOptimizer::setCancelCallback(const std::function<bool()>& cancel_callback)
{
    cancel_callback_ = cancel_callback;
}

bool MincoGcopterOptimizer::optimize(const BoundaryState& start,
                                     const BoundaryState& goal,
                                     const std::vector<Eigen::MatrixX4d>& corridors,
                                     MincoTraj& output_traj)
{
    last_error_.clear();
    last_cost_ = 0.0;
    output_traj.clear();

    if (!config_.enable)
    {
        last_error_ = "MINCO/GCOPTER optimizer is disabled.";
        return false;
    }

    if (!validateInputs(corridors))
    {
        return false;
    }

    gcopter::GCOPTER_PolytopeSFC optimizer;
    const Eigen::Matrix3d ini_state = makePvaMatrix(start);
    const Eigen::Matrix3d fin_state = makePvaMatrix(goal);
    const Eigen::VectorXd magnitude_bounds = makeMagnitudeBounds();
    const Eigen::VectorXd penalty_weights = makePenaltyWeights();
    const Eigen::VectorXd physical_params = makePhysicalParams();

    // GCOPTER 会在 setup() 内把 H-polytope 转成 V-polytope，并根据走廊长度分配 MINCO piece。
    const bool setup_ok = optimizer.setup(config_.weight_time,
                                          ini_state,
                                          fin_state,
                                          corridors,
                                          config_.length_per_piece,
                                          config_.smoothing_eps,
                                          config_.integral_intervals,
                                          magnitude_bounds,
                                          penalty_weights,
                                          physical_params);
    if (!setup_ok)
    {
        last_error_ = "GCOPTER setup failed. Corridor may be invalid or empty.";
        return false;
    }
    optimizer.setCancelCallback(cancel_callback_);

    MincoTraj::TrajectoryType raw_traj;
    const double cost = optimizer.optimize(raw_traj, config_.rel_cost_tol);
    last_cost_ = cost;
    if (!std::isfinite(cost) || raw_traj.getPieceNum() <= 0)
    {
        last_error_ = "GCOPTER optimization failed.";
        return false;
    }

    output_traj.setTrajectory(raw_traj);
    if (!output_traj.valid())
    {
        last_error_ = "GCOPTER returned an invalid trajectory.";
        return false;
    }

    return true;
}

std::string MincoGcopterOptimizer::lastError() const
{
    return last_error_;
}

double MincoGcopterOptimizer::lastCost() const
{
    return last_cost_;
}

Eigen::Matrix3d MincoGcopterOptimizer::makePvaMatrix(const BoundaryState& state) const
{
    Eigen::Matrix3d pva;
    // GCOPTER 约定矩阵列为 $[p, v, a]$。
    pva.col(0) = state.pos;
    pva.col(1) = state.vel;
    pva.col(2) = state.acc;
    return pva;
}

Eigen::VectorXd MincoGcopterOptimizer::makeMagnitudeBounds() const
{
    Eigen::VectorXd bounds(5);
    bounds(0) = config_.max_vel;
    bounds(1) = config_.max_body_rate;
    bounds(2) = config_.max_tilt_angle;
    bounds(3) = config_.min_thrust;
    bounds(4) = config_.max_thrust;
    return bounds;
}

Eigen::VectorXd MincoGcopterOptimizer::makePenaltyWeights() const
{
    Eigen::VectorXd weights(5);
    weights(0) = config_.penalty_pos;
    weights(1) = config_.penalty_vel;
    weights(2) = config_.penalty_body_rate;
    weights(3) = config_.penalty_tilt;
    weights(4) = config_.penalty_thrust;
    return weights;
}

Eigen::VectorXd MincoGcopterOptimizer::makePhysicalParams() const
{
    Eigen::VectorXd params(6);
    params(0) = config_.mass;
    params(1) = config_.gravity;
    params(2) = config_.horiz_drag;
    params(3) = config_.vert_drag;
    params(4) = config_.paras_drag;
    params(5) = config_.speed_eps;
    return params;
}

bool MincoGcopterOptimizer::validateInputs(const std::vector<Eigen::MatrixX4d>& corridors)
{
    if (corridors.empty())
    {
        last_error_ = "Safe corridor is empty.";
        return false;
    }

    for (const Eigen::MatrixX4d& hpoly : corridors)
    {
        if (hpoly.cols() != 4 || hpoly.rows() < 4)
        {
            last_error_ = "Invalid H-polytope shape.";
            return false;
        }
        if (!hpoly.allFinite())
        {
            last_error_ = "H-polytope contains non-finite value.";
            return false;
        }
    }

    return true;
}

}  // namespace fastnav
