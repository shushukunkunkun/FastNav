/*
 * planner_debug_recorder.h
 *
 * 本文件声明 PlannerDebugRecorder。
 * 该类只在 debug 模式下工作，用于把一次规划失败时的“现场”写入磁盘：
 * 1. 当前输入点云、planner 内部 occupied / inflated debug cloud；
 * 2. 前端 A* 路径、后端优化参考路径、shortcut 路径和 MINCO 采样轨迹；
 * 3. safe corridor 半空间 $n^T x + d \le 0$；
 * 4. 起点、目标点、实际规划终点、失败原因和耗时统计。
 *
 * 它不参与规划闭环，不发布 ROS topic，只服务于离线复盘和 RViz / Python 可视化。
 */

#pragma once

#include <string>
#include <vector>

#include <Eigen/Core>
#include <ros/time.h>
#include <sensor_msgs/PointCloud2.h>

#include <traj_utils/minco/minco_traj.h>

namespace fastnav_planner
{

class PlannerDebugRecorder
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    struct Config
    {
        bool enable{false};
        std::string output_dir{"/home/shukun/Project/FastNav/tools/debug_cases"};
        int max_cases{200};
        bool record_cloud{true};
        bool record_voxel_map{true};
        bool record_corridor{true};
        bool record_minco_samples{true};
        double minco_sample_dt{0.05};
        std::string frame_id{"odom"};
    };

    struct Timing
    {
        double guide_astar_ms{0.0};
        double frontend_astar_ms{0.0};
        double reference_ms{0.0};
        double shortcut_ms{0.0};
        double corridor_ms{0.0};
        double minco_ms{0.0};
        double fine_check_ms{0.0};
        int astar_nodes{0};
        int corridor_num{0};
        int minco_retry_count{0};
        double clearance_used{0.0};
    };

    struct Snapshot
    {
        ros::Time stamp;
        std::string frame_id{"odom"};
        std::string state;
        std::string reason;

        int attempt{0};
        int continuous_failures{0};
        bool use_current_traj{false};
        bool use_random_init{false};
        bool touch_goal{true};
        bool reached_requested_goal{true};

        Eigen::Vector3d odom_pos{Eigen::Vector3d::Zero()};
        Eigen::Vector3d start{Eigen::Vector3d::Zero()};
        Eigen::Vector3d requested_goal{Eigen::Vector3d::Zero()};
        Eigen::Vector3d planned_target{Eigen::Vector3d::Zero()};

        Timing timing;

        std::vector<Eigen::Vector3d> frontend_path;
        std::vector<Eigen::Vector3d> reference_path;
        std::vector<Eigen::Vector3d> shortcut_path;
        std::vector<Eigen::Vector3d> sampled_path;
        std::vector<Eigen::Vector3d> searched_nodes;
        std::vector<Eigen::MatrixX4d> corridors;

        bool has_minco{false};
        fastnav::MincoTraj minco_traj;

        bool has_filtered_cloud{false};
        bool has_occupied_cloud{false};
        bool has_inflated_cloud{false};
        sensor_msgs::PointCloud2 filtered_cloud;
        sensor_msgs::PointCloud2 occupied_cloud;
        sensor_msgs::PointCloud2 inflated_cloud;
    };

    void setConfig(const Config& config);
    bool enabled() const { return config_.enable; }
    std::string lastCaseDir() const { return last_case_dir_; }

    // 将一次失败快照写入 output_dir。返回 false 表示未启用、达到上限或写文件失败。
    bool recordMincoFailure(const Snapshot& snapshot);

private:
    bool ensureDirectory(const std::string& path) const;
    std::string createCaseDirectory(const Snapshot& snapshot);
    bool writeMeta(const std::string& dir, const Snapshot& snapshot) const;
    bool writePathCsv(const std::string& path,
                      const std::vector<Eigen::Vector3d>& points) const;
    bool writeCorridorCsv(const std::string& path,
                          const std::vector<Eigen::MatrixX4d>& corridors) const;
    bool writeMincoSamplesCsv(const std::string& path,
                              const fastnav::MincoTraj& traj,
                              double sample_dt) const;
    bool writeCloudPcd(const std::string& path,
                       const sensor_msgs::PointCloud2& cloud) const;

private:
    Config config_;
    int recorded_cases_{0};
    std::string last_case_dir_;
};

}  // namespace fastnav_planner
