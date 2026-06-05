#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Eigen>

#include <traj_utils/minco/gcopter/trajectory.hpp>

namespace fastnav
{

// FastNav 对 GCOPTER Trajectory<5> 的工程层封装。
// GCOPTER 的 Trajectory<5> 是底层数学轨迹，表示分段五次多项式 $p_i(t)$；MincoTraj 负责提供更稳定的访问接口。
class MincoTraj
{
public:
    using TrajectoryType = ::Trajectory<5>;

    struct DynamicFeasibilityResult
    {
        bool feasible{false};
        std::string violation_type;
        double first_violation_time{0.0};
        double max_vel{0.0};
        double max_acc{0.0};
        double max_jerk{0.0};
        Eigen::Vector3d value{Eigen::Vector3d::Zero()};

        void reset()
        {
            feasible = false;
            violation_type.clear();
            first_violation_time = 0.0;
            max_vel = 0.0;
            max_acc = 0.0;
            max_jerk = 0.0;
            value.setZero();
        }
    };

    MincoTraj() = default;
    explicit MincoTraj(const TrajectoryType& traj) { setTrajectory(traj); }

    void clear()
    {
        traj_.clear();
        valid_ = false;
    }

    void setTrajectory(const TrajectoryType& traj)
    {
        traj_ = traj;
        valid_ = traj_.getPieceNum() > 0 && traj_.getTotalDuration() > 1.0e-6;
    }

    bool valid() const { return valid_; }
    bool empty() const { return !valid_; }

    const TrajectoryType& trajectory() const { return traj_; }
    TrajectoryType& mutableTrajectory() { return traj_; }

    double getDuration() const
    {
        return valid_ ? traj_.getTotalDuration() : 0.0;
    }

    int getPieceNum() const
    {
        return valid_ ? traj_.getPieceNum() : 0;
    }

    double getMaxVelRate() const
    {
        return valid_ ? traj_.getMaxVelRate() : 0.0;
    }

    double getMaxAccRate() const
    {
        return valid_ ? traj_.getMaxAccRate() : 0.0;
    }

    bool checkMaxVelRate(double max_vel) const
    {
        if (!valid_ || max_vel <= 0.0)
        {
            return false;
        }
        return traj_.checkMaxVelRate(max_vel);
    }

    bool checkMaxAccRate(double max_acc) const
    {
        if (!valid_ || max_acc <= 0.0)
        {
            return false;
        }
        return traj_.checkMaxAccRate(max_acc);
    }

    // 位置采样。输入时间 $t$ 是轨迹局部时间，范围会被夹紧到 $[0,T]$。
    Eigen::Vector3d getPosition(double t) const
    {
        if (!valid_)
        {
            return Eigen::Vector3d::Zero();
        }
        return traj_.getPos(clampTime(t));
    }

    // 速度采样，返回 $\dot p(t)$。
    Eigen::Vector3d getVelocity(double t) const
    {
        if (!valid_)
        {
            return Eigen::Vector3d::Zero();
        }
        return traj_.getVel(clampTime(t));
    }

    // 加速度采样，返回 $\ddot p(t)$。
    Eigen::Vector3d getAcceleration(double t) const
    {
        if (!valid_)
        {
            return Eigen::Vector3d::Zero();
        }
        return traj_.getAcc(clampTime(t));
    }

    // jerk 采样，返回 $p^{(3)}(t)$。
    Eigen::Vector3d getJerk(double t) const
    {
        if (!valid_)
        {
            return Eigen::Vector3d::Zero();
        }
        return traj_.getJer(clampTime(t));
    }

    // 按固定时间步长采样位置点，常用于生成 nav_msgs::Path 或 RViz 调试路径。
    std::vector<Eigen::Vector3d> samplePositions(double dt) const
    {
        std::vector<Eigen::Vector3d> samples;
        if (!valid_)
        {
            return samples;
        }

        const double step = std::max(1.0e-3, dt);
        const double duration = getDuration();
        const int sample_num = std::max(1, static_cast<int>(std::ceil(duration / step)));
        samples.reserve(sample_num + 1);
        for (int i = 0; i <= sample_num; ++i)
        {
            const double t = std::min(duration, static_cast<double>(i) * step);
            samples.push_back(getPosition(t));
        }
        return samples;
    }

    // 动力学可行性检查。速度和加速度优先使用 Trajectory<5> 的解析根求解结果，
    // jerk 暂按时间步长采样检查 $||p^{(3)}(t)|| \le j_{max}(1+\epsilon)$。
    bool checkDynamicFeasibility(double max_vel,
                                 double max_acc,
                                 double max_jerk,
                                 double tolerance,
                                 double jerk_sample_dt,
                                 DynamicFeasibilityResult* result = nullptr) const
    {
        DynamicFeasibilityResult local_result;
        local_result.reset();
        if (!valid_)
        {
            local_result.violation_type = "invalid_traj";
            if (result)
            {
                *result = local_result;
            }
            return false;
        }

        const double scale = 1.0 + std::max(0.0, tolerance);
        const double vel_limit = max_vel > 0.0 ? max_vel * scale : std::numeric_limits<double>::infinity();
        const double acc_limit = max_acc > 0.0 ? max_acc * scale : std::numeric_limits<double>::infinity();
        const double jerk_limit = max_jerk > 0.0 ? max_jerk * scale : std::numeric_limits<double>::infinity();

        local_result.max_vel = getMaxVelRate();
        local_result.max_acc = getMaxAccRate();

        if (max_vel > 0.0 && !traj_.checkMaxVelRate(vel_limit))
        {
            local_result.violation_type = "velocity";
            local_result.value = getVelocity(findFirstNormViolationTime(
                [this](double t) { return getVelocity(t); }, vel_limit, jerk_sample_dt));
            local_result.first_violation_time = findFirstNormViolationTime(
                [this](double t) { return getVelocity(t); }, vel_limit, jerk_sample_dt);
            if (result)
            {
                *result = local_result;
            }
            return false;
        }

        if (max_acc > 0.0 && !traj_.checkMaxAccRate(acc_limit))
        {
            local_result.violation_type = "acceleration";
            local_result.first_violation_time = findFirstNormViolationTime(
                [this](double t) { return getAcceleration(t); }, acc_limit, jerk_sample_dt);
            local_result.value = getAcceleration(local_result.first_violation_time);
            if (result)
            {
                *result = local_result;
            }
            return false;
        }

        const double step = std::max(1.0e-3, jerk_sample_dt);
        const double duration = getDuration();
        const int sample_num = std::max(1, static_cast<int>(std::ceil(duration / step)));
        for (int i = 0; i <= sample_num; ++i)
        {
            const double t = std::min(duration, static_cast<double>(i) * step);
            const Eigen::Vector3d jerk = getJerk(t);
            const double jerk_norm = jerk.norm();
            local_result.max_jerk = std::max(local_result.max_jerk, jerk_norm);
            if (max_jerk > 0.0 && jerk_norm > jerk_limit)
            {
                local_result.violation_type = "jerk";
                local_result.first_violation_time = t;
                local_result.value = jerk;
                if (result)
                {
                    *result = local_result;
                }
                return false;
            }
        }

        local_result.feasible = true;
        if (result)
        {
            *result = local_result;
        }
        return true;
    }

private:
    double clampTime(double t) const
    {
        const double duration = getDuration();
        return std::min(duration, std::max(0.0, t));
    }

    template <typename VectorFunc>
    double findFirstNormViolationTime(VectorFunc&& func,
                                      double limit,
                                      double sample_dt) const
    {
        const double step = std::max(1.0e-3, sample_dt);
        const double duration = getDuration();
        const int sample_num = std::max(1, static_cast<int>(std::ceil(duration / step)));
        for (int i = 0; i <= sample_num; ++i)
        {
            const double t = std::min(duration, static_cast<double>(i) * step);
            if (func(t).norm() > limit)
            {
                return t;
            }
        }
        return duration;
    }

private:
    TrajectoryType traj_;
    bool valid_{false};
};

}  // namespace fastnav
