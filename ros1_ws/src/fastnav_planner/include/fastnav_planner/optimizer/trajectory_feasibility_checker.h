#pragma once

#include <memory>
#include <string>

#include <Eigen/Core>

#include <fastnav_mapping/voxel_map.h>
#include <traj_utils/minco/minco_traj.h>

namespace fastnav_planner
{

// TrajectoryFeasibilityChecker 是 MINCO 优化后的验收层。
// 它对应 EGO-Planner-v2 中 L-BFGS 结束后的 fine check：只有轨迹满足地图碰撞约束和动力学约束，planner 才接受它。
class TrajectoryFeasibilityChecker
{
public:
    struct Config
    {
        bool check_collision{true};
        bool check_dynamics{true};

        // 碰撞采样时间间隔。轨迹位置 $p(t)$ 会按 $t_k=k\Delta t$ 离散检查。
        double sample_dt{0.03};
        // 相邻采样点之间仍用 VoxelMap::isLineFree() 做空间线段检查，避免高速段跨过薄障碍。
        double collision_check_step{0.1};
        // 小于 1 时只检查前段轨迹，类似 EGO 只检查局部轨迹靠近无人机的前 $2/3$。
        double check_horizon_ratio{1.0};

        double max_vel{2.0};
        double max_acc{3.0};
        double max_jerk{4.0};
        double tolerance{0.05};
    };

    struct Result
    {
        bool feasible{false};
        std::string violation_type;
        std::string message;
        double violation_time{0.0};
        Eigen::Vector3d position{Eigen::Vector3d::Zero()};
        Eigen::Vector3d value{Eigen::Vector3d::Zero()};
        double max_vel{0.0};
        double max_acc{0.0};
        double max_jerk{0.0};

        void reset();
    };

    void setConfig(const Config& config);

    bool check(const fastnav::MincoTraj& traj,
               const std::shared_ptr<fastnav_mapping::VoxelMap>& map,
               Result& result) const;

private:
    bool checkCollision(const fastnav::MincoTraj& traj,
                        const std::shared_ptr<fastnav_mapping::VoxelMap>& map,
                        Result& result) const;

    bool checkDynamics(const fastnav::MincoTraj& traj,
                       Result& result) const;

    std::string formatViolation(const Result& result) const;

private:
    Config config_;
};

}  // namespace fastnav_planner
