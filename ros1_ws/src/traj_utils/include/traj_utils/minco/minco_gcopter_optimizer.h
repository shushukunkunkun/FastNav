#pragma once

#include <functional>
#include <string>
#include <vector>

#include <Eigen/Eigen>

#include <traj_utils/minco/minco_traj.h>

namespace fastnav
{

// MincoGcopterOptimizer 把 FastNav 的边界状态和安全走廊转换成 GCOPTER 优化问题。
// 它隐藏 GCOPTER_PolytopeSFC 的复杂参数，使 planner 只需要传入 $[p,v,a]$、走廊和配置。
class MincoGcopterOptimizer
{
public:
    struct BoundaryState
    {
        Eigen::Vector3d pos{Eigen::Vector3d::Zero()};
        Eigen::Vector3d vel{Eigen::Vector3d::Zero()};
        Eigen::Vector3d acc{Eigen::Vector3d::Zero()};
    };

    struct Config
    {
        bool enable{true};

        // GCOPTER 时间代价权重，目标中包含 $rho * sum(T_i)$。
        double weight_time{1.0};
        // 平滑 hinge 惩罚的光滑系数，用于约束违反 $max(0,g(x))$ 的近似。
        double smoothing_eps{1.0e-2};
        // 每段轨迹积分采样数量，用于评估速度、倾角、推力等约束。
        int integral_intervals{16};
        // L-BFGS 相对代价收敛阈值。
        double rel_cost_tol{1.0e-5};
        // 每个 MINCO piece 期望长度；GCOPTER 内部会据此把 corridor 分段成多个 piece。
        double length_per_piece{1.0};

        // magnitudeBounds = [v_max, omega_max, theta_max, thrust_min, thrust_max]^T。
        double max_vel{1.5};
        double max_body_rate{2.0};
        double max_tilt_angle{0.52};
        double min_thrust{2.0};
        double max_thrust{20.0};

        // physicalParams = [mass, gravity, horiz_drag, vert_drag, paras_drag, speed_eps]^T。
        double mass{1.5};
        double gravity{9.81};
        double horiz_drag{0.0};
        double vert_drag{0.0};
        double paras_drag{0.0};
        double speed_eps{1.0e-2};

        // penaltyWeights = [pos, vel, omega, theta, thrust]^T。
        double penalty_pos{1.0e4};
        double penalty_vel{1.0e3};
        double penalty_body_rate{1.0e3};
        double penalty_tilt{1.0e3};
        double penalty_thrust{1.0e3};
    };

    void setConfig(const Config& config);
    void setCancelCallback(const std::function<bool()>& cancel_callback);

    bool optimize(const BoundaryState& start,
                  const BoundaryState& goal,
                  const std::vector<Eigen::MatrixX4d>& corridors,
                  MincoTraj& output_traj);

    std::string lastError() const;
    double lastCost() const;

private:
    Eigen::Matrix3d makePvaMatrix(const BoundaryState& state) const;
    Eigen::VectorXd makeMagnitudeBounds() const;
    Eigen::VectorXd makePenaltyWeights() const;
    Eigen::VectorXd makePhysicalParams() const;
    bool validateInputs(const std::vector<Eigen::MatrixX4d>& corridors);

private:
    Config config_;
    std::function<bool()> cancel_callback_;
    std::string last_error_;
    double last_cost_{0.0};
};

}  // namespace fastnav
