#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>

#include <fastnav_mapping/voxel_map.h>
#include <traj_utils/corridor/safe_corridor_generator.h>
#include <traj_utils/minco/minco_gcopter_optimizer.h>

#include "fastnav_planner/optimizer/trajectory_feasibility_checker.h"

namespace fastnav_planner
{

// PathOptimizer 是 planner 内部的纯算法优化器类，不订阅 ROS topic，也不生成控制量。
// 它把 A* 折线路径 $r_k$ 先做 shortcut，再生成安全走廊 $H_i[x^T,1]^T\le0$，最后调用 GCOPTER/MINCO 得到连续轨迹 $p(t)$。
class PathOptimizer
{
public:
    struct Config
    {
        bool enable{true};
        bool shortcut{true};
        double line_check_step{0.1};

        bool minco_enable{true};
        bool require_minco{true};
        double minco_sample_dt{0.05};
        int max_retry{3};

        // 失败重试时的参数调节。思路来自 EGO-v2：不改变轨迹表达，而是改变约束和初始化条件后重新优化。
        double corridor_range_retry_scale{1.2};
        double corridor_progress_retry_scale{1.0};
        double weight_time_retry_scale{1.0};
        double penalty_pos_retry_scale{2.0};
        double penalty_vel_retry_scale{1.5};
        double penalty_body_rate_retry_scale{1.0};
        double penalty_tilt_retry_scale{1.0};
        double penalty_thrust_retry_scale{1.0};

        traj_utils::SafeCorridorGenerator::Config corridor;
        fastnav::MincoGcopterOptimizer::Config minco;
        TrajectoryFeasibilityChecker::Config feasibility;
    };

    struct OptimizationResult
    {
        bool has_minco{false};
        std::vector<Eigen::Vector3d> raw_path;
        std::vector<Eigen::Vector3d> shortcut_path;
        std::vector<Eigen::Vector3d> sampled_path;
        std::vector<Eigen::MatrixX4d> corridors;
        fastnav::MincoTraj minco_traj;
        double shortcut_ms{0.0};
        double corridor_ms{0.0};
        double minco_ms{0.0};
        double fine_check_ms{0.0};
        int minco_retry_count{0};

        void clear()
        {
            has_minco = false;
            raw_path.clear();
            shortcut_path.clear();
            sampled_path.clear();
            corridors.clear();
            minco_traj.clear();
            shortcut_ms = 0.0;
            corridor_ms = 0.0;
            minco_ms = 0.0;
            fine_check_ms = 0.0;
            minco_retry_count = 0;
        }
    };

    void setConfig(const Config& config);

    bool optimizeTrajectory(const std::vector<Eigen::Vector3d>& raw_path,
                            const std::shared_ptr<fastnav_mapping::VoxelMap>& map,
                            const fastnav::MincoGcopterOptimizer::BoundaryState& start_state,
                            const fastnav::MincoGcopterOptimizer::BoundaryState& goal_state,
                            OptimizationResult& result,
                            size_t preserve_prefix_size = 0,
                            bool touch_goal = true,
                            const std::function<bool()>& preempt_requested = std::function<bool()>());

    // 兼容旧接口：只输出几何路径；如果 MINCO 成功则输出采样路径，否则输出 shortcut 路径。
    bool optimize(const std::vector<Eigen::Vector3d>& raw_path,
                  const std::shared_ptr<fastnav_mapping::VoxelMap>& map,
                  std::vector<Eigen::Vector3d>& optimized_path);

    std::string lastError() const;

private:
    bool shortcutPath(const std::vector<Eigen::Vector3d>& raw_path,
                      const std::shared_ptr<fastnav_mapping::VoxelMap>& map,
                      std::vector<Eigen::Vector3d>& optimized_path,
                      size_t preserve_prefix_size = 0) const;

    void collectSurfacePointsForPath(const std::vector<Eigen::Vector3d>& path,
                                     const std::shared_ptr<fastnav_mapping::VoxelMap>& map,
                                     double range,
                                     std::vector<Eigen::Vector3d>& obstacle_surface_points) const;

    bool generateCorridor(const std::vector<Eigen::Vector3d>& path,
                          const std::shared_ptr<fastnav_mapping::VoxelMap>& map,
                          const std::vector<Eigen::Vector3d>& obstacle_surface_points,
                          std::vector<Eigen::MatrixX4d>& corridors);

    bool shouldRegenerateCorridorAfterViolation(const std::string& violation_type) const;

private:
    Config config_;
    traj_utils::SafeCorridorGenerator corridor_generator_;
    fastnav::MincoGcopterOptimizer minco_optimizer_;
    TrajectoryFeasibilityChecker feasibility_checker_;
    std::string last_error_;
};

}  // namespace fastnav_planner
