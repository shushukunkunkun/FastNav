#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>

#include <fastnav_mapping/voxel_map.h>

namespace fastnav_planner
{

// AStarPlanner 是纯算法类，不订阅 ROS topic。
// 搜索过程中直接调用 VoxelMap 查询接口，例如 $map->isInflatedOccupied(p)$，避免 node/service 高频通信。
class AStarPlanner
{
public:
    struct Config
    {
        bool allow_diagonal{true};
        bool check_line_collision{true};
        double heuristic_weight{1.2};
        double line_check_step{0.1};
        // frontend map 相对基础 inflated map 的额外膨胀半径。A* 本身只做 O(1) map 查询，
        // 不再运行时扫描 clearance 邻域；$min_clearance=0$ 时 frontend map 退化为基础 map。
        double min_clearance{0.0};
        // 以下两个参数为旧版运行时 clearance relaxation 预留，当前不再用于邻域扫描。
        double clearance_retry_scale{0.7};
        double min_clearance_floor{0.0};
        int max_search_nodes{50000};
        // SUPER / EGO 风格前端硬截断，避免 A* 长时间阻塞 FSM heartbeat。
        double max_search_time{0.2};
    };

    void setMap(const std::shared_ptr<fastnav_mapping::VoxelMap>& map);
    void setConfig(const Config& config);
    void setCancelCallback(const std::function<bool()>& cancel_callback);

    bool plan(const Eigen::Vector3d& start,
              const Eigen::Vector3d& goal,
              std::vector<Eigen::Vector3d>& path);

    // 沿最终目标方向做带 clearance 的局部 guide search。
    // 搜索不要求抵达 final_goal；当累计路径长度 $g \ge horizon$ 时停止，并返回 $start -> local_target$。
    bool planToHorizon(const Eigen::Vector3d& start,
                       const Eigen::Vector3d& final_goal,
                       double horizon,
                       std::vector<Eigen::Vector3d>& path);

    const std::vector<Eigen::Vector3d>& searchedNodes() const;
    std::string lastError() const;
    int lastExpandedNodes() const { return last_expanded_nodes_; }
    int lastAttemptCount() const { return last_attempt_count_; }
    double lastClearanceUsed() const { return last_clearance_used_; }

private:
    struct NodeRecord
    {
        double g{0.0};
        double f{0.0};
        int parent{-1};
        bool closed{false};
    };

    int toAddress(const Eigen::Vector3i& idx) const;
    Eigen::Vector3i addressToIndex(int address) const;
    double heuristic(const Eigen::Vector3i& current, const Eigen::Vector3i& goal) const;
    bool planOnFrontendMap(const Eigen::Vector3d& start,
                           const Eigen::Vector3d& goal,
                           std::vector<Eigen::Vector3d>& path);
    bool planToHorizonOnFrontendMap(const Eigen::Vector3d& start,
                                    const Eigen::Vector3d& final_goal,
                                    double horizon,
                                    std::vector<Eigen::Vector3d>& path);
    std::vector<Eigen::Vector3i> neighborOffsets() const;

private:
    std::shared_ptr<fastnav_mapping::VoxelMap> map_;
    Config config_;
    std::function<bool()> cancel_callback_;
    std::vector<Eigen::Vector3d> searched_nodes_;
    std::string last_error_;
    int last_expanded_nodes_{0};
    int last_attempt_count_{0};
    double last_clearance_used_{0.0};
};

}  // namespace fastnav_planner
