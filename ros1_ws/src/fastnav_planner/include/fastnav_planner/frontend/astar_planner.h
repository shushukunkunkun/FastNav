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
        // inflated map 之外的额外安全余量。$min_clearance=0$ 时只使用 inflated occupancy。
        double min_clearance{0.0};
        int max_search_nodes{50000};
    };

    void setMap(const std::shared_ptr<fastnav_mapping::VoxelMap>& map);
    void setConfig(const Config& config);
    void setCancelCallback(const std::function<bool()>& cancel_callback);

    bool plan(const Eigen::Vector3d& start,
              const Eigen::Vector3d& goal,
              std::vector<Eigen::Vector3d>& path);

    const std::vector<Eigen::Vector3d>& searchedNodes() const;
    std::string lastError() const;

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
    bool hasExtraClearance(const Eigen::Vector3d& pos) const;
    bool isSegmentClearWithExtraClearance(const Eigen::Vector3d& p0,
                                          const Eigen::Vector3d& p1,
                                          bool relax_start_clearance,
                                          bool relax_goal_clearance) const;
    std::vector<Eigen::Vector3i> neighborOffsets() const;

private:
    std::shared_ptr<fastnav_mapping::VoxelMap> map_;
    Config config_;
    std::function<bool()> cancel_callback_;
    std::vector<Eigen::Vector3d> searched_nodes_;
    std::string last_error_;
};

}  // namespace fastnav_planner
