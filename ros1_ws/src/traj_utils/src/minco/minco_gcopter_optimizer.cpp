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

// 调用 GCOPTER 后端，在 safe corridor 中求解一条 MINCO 轨迹。
// 输入：
// - start / goal 分别提供端点边界状态 $[p_0,v_0,a_0]$ 和 $[p_T,v_T,a_T]$；
// - corridors 是 FIRI 生成的 H-polytope 序列，每个多面体形如 $H_k[x^T,1]^T \le 0$；
// 输出：
// - output_traj 保存分段多项式轨迹 $p(t)$，后续 traj_server 会按时间采样它生成控制指令。
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

    // GCOPTER_PolytopeSFC 是原 GCOPTER 的核心优化器：
    // 它接收端点 PVA、走廊多面体、动力学限制和惩罚权重，
    // 内部构造 MINCO 多项式参数化，并用 L-BFGS 优化轨迹时间和中间状态。
    gcopter::GCOPTER_PolytopeSFC optimizer;

    // ini_state / fin_state 的列顺序为 $[p,v,a]$。
    // 这些边界条件会成为 MINCO 的等式约束，使轨迹满足
    // $p(0)=p_0,v(0)=v_0,a(0)=a_0$ 和 $p(T)=p_T,v(T)=v_T,a(T)=a_T$。
    const Eigen::Matrix3d ini_state = makePvaMatrix(start);
    const Eigen::Matrix3d fin_state = makePvaMatrix(goal);

    // magnitude_bounds 保存硬/软约束上限：
    // $v_{max}$、body rate、tilt、最小/最大 thrust。
    // penalty_weights 则是违反这些约束时加入目标函数的惩罚权重。
    const Eigen::VectorXd magnitude_bounds = makeMagnitudeBounds();
    const Eigen::VectorXd penalty_weights = makePenaltyWeights();

    // physical_params 包含质量、重力和阻力参数，用于 GCOPTER 的微分平坦性动力学模型。
    const Eigen::VectorXd physical_params = makePhysicalParams();

    // GCOPTER 会在 setup() 内把 H-polytope 转成 V-polytope，并根据走廊长度分配 MINCO piece。
    // length_per_piece 控制每段 corridor 内大致放多少 MINCO piece；
    // smoothing_eps 和 integral_intervals 控制 penalty 积分的平滑近似与离散积分精度。
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

    // optimize() 内部执行 L-BFGS。目标函数可以理解为：
    // $J = J_{smooth} + w_T T + \lambda_p J_{corridor} + \lambda_v J_{vel} + \lambda_\omega J_{bodyrate} + \lambda_\theta J_{tilt} + \lambda_f J_{thrust}$
    // rel_cost_tol 是相对代价下降阈值，代价收敛后停止迭代。
    const double cost = optimizer.optimize(raw_traj, config_.rel_cost_tol);
    last_cost_ = cost;
    if (!std::isfinite(cost) || raw_traj.getPieceNum() <= 0)
    {
        last_error_ = "GCOPTER optimization failed.";
        return false;
    }

    // 将 GCOPTER 原生 Trajectory 封装进 FastNav 的 MincoTraj，
    // 这样上层只依赖 traj_utils 的统一轨迹接口，例如 getPosition(t)、getVelocity(t)。
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
