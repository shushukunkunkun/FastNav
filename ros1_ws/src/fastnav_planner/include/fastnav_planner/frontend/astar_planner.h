#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <ros/time.h>

#include <fastnav_mapping/voxel_map.h>

namespace fastnav_planner
{

// AStarPlanner 是纯算法类，不订阅 ROS topic。
// 搜索过程中直接调用 VoxelMap 查询接口，例如 $map->isInflatedOccupied(p)$，避免 node/service 高频通信。
class AStarPlanner
{
public:
    enum class SearchStatus
    {
        SUCCESS,
        REACH_GOAL,
        REACH_HORIZON,
        BEST_EFFORT,
        TIME_OUT,
        NO_PATH,
        INIT_ERROR,
        PREEMPTED
    };

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

        // 若 goal 在局部地图外，将射线 $p(s)=p_0+s(p_g-p_0)$ 与地图 AABB 求交，
        // 把本次搜索终点投影到地图内，避免 outside local map 直接终止。
        bool enable_goal_projection{true};
        int projection_margin_voxels{2};
        double nearest_free_search_radius{1.5};

        // 若前端在时间限制内没有到达目标/视野边界，允许返回当前搜索树中最接近目标的节点。
        bool allow_timeout_best_effort{true};
        double timeout_best_effort_min_length{1.0};
        // guide search 超时时，manager 会把 horizon 缩短为 $H'=\max(H_{min},\alpha H)$ 后再试一次。
        double timeout_horizon_scale{0.6};
        double timeout_min_horizon{1.5};

        // Escape search 使用基础 map 通行、frontend map 作为“逃离目标”，寻找第一个满足
        // $p \notin \mathcal{O}_{front}$ 的点，解决起点贴近保守膨胀层时 A* 出不去的问题。
        bool enable_escape_search{true};
        double escape_max_radius{1.5};
        double escape_max_search_time{0.05};
        int escape_max_nodes{3000};
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

    bool escapeFromInflatedRegion(const Eigen::Vector3d& start,
                                  const std::shared_ptr<fastnav_mapping::VoxelMap>& blocking_map,
                                  std::vector<Eigen::Vector3d>& path);

    const std::vector<Eigen::Vector3d>& searchedNodes() const;
    std::string lastError() const;
    SearchStatus lastStatus() const { return last_status_; }
    Eigen::Vector3d lastPathTarget() const { return last_path_target_; }
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
    bool isTimeOut(const ros::WallTime& search_start, double limit) const;
    bool isPreempted() const;
    bool retrievePath(const std::vector<NodeRecord>& records,
                      int start_addr,
                      int end_addr,
                      const Eigen::Vector3d& start,
                      const Eigen::Vector3d& end,
                      std::vector<Eigen::Vector3d>& path) const;
    Eigen::Vector3d clampInsideMap(const Eigen::Vector3d& pos, double margin) const;
    bool projectGoalIntoMap(const Eigen::Vector3d& start,
                            const Eigen::Vector3d& goal,
                            Eigen::Vector3d& projected_goal,
                            bool& was_projected) const;
    bool findNearestFreePosition(const Eigen::Vector3d& seed,
                                 double search_radius,
                                 Eigen::Vector3d& free_pos) const;
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
    SearchStatus last_status_{SearchStatus::NO_PATH};
    Eigen::Vector3d last_path_target_{Eigen::Vector3d::Zero()};
    int last_expanded_nodes_{0};
    int last_attempt_count_{0};
    double last_clearance_used_{0.0};
};

}  // namespace fastnav_planner
