#include "fastnav_planner/frontend/astar_planner.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>

#include <ros/time.h>

namespace fastnav_planner
{

void AStarPlanner::setMap(const std::shared_ptr<fastnav_mapping::VoxelMap>& map)
{
    map_ = map;
}

void AStarPlanner::setConfig(const Config& config)
{
    config_ = config;
    config_.min_clearance = std::max(0.0, config_.min_clearance);
    config_.min_clearance_floor = std::max(0.0, std::min(config_.min_clearance_floor, config_.min_clearance));
    config_.clearance_retry_scale = std::min(0.99, std::max(0.05, config_.clearance_retry_scale));
    config_.max_search_time = std::max(0.0, config_.max_search_time);
}

void AStarPlanner::setCancelCallback(const std::function<bool()>& cancel_callback)
{
    cancel_callback_ = cancel_callback;
}

bool AStarPlanner::plan(const Eigen::Vector3d& start,
                        const Eigen::Vector3d& goal,
                        std::vector<Eigen::Vector3d>& path)
{
    last_attempt_count_ = 0;
    last_clearance_used_ = config_.min_clearance;
    ++last_attempt_count_;

    if (planOnFrontendMap(start, goal, path))
    {
        return true;
    }

    if (cancel_callback_ && cancel_callback_())
    {
        return false;
    }

    last_error_ = "A* failed on current inflated map: " + last_error_;
    return false;
}

bool AStarPlanner::planOnFrontendMap(const Eigen::Vector3d& start,
                                     const Eigen::Vector3d& goal,
                                     std::vector<Eigen::Vector3d>& path)
{
    path.clear();
    searched_nodes_.clear();
    last_error_.clear();

    if (!map_)
    {
        last_error_ = "VoxelMap is not set.";
        return false;
    }

    Eigen::Vector3i start_idx;
    Eigen::Vector3i goal_idx;
    if (!map_->posToIndex(start, start_idx))
    {
        last_error_ = "Start is outside local map.";
        return false;
    }
    if (!map_->posToIndex(goal, goal_idx))
    {
        last_error_ = "Goal is outside local map.";
        return false;
    }

    // frontend map 比基础 map 更保守，start/goal 可能只落在“额外 clearance”区域。
    // 这里允许 A* 从 start 逃出、允许最后落到 goal；真实碰撞由后端基础 map fine check 兜底。

    const Eigen::Vector3i dim = map_->dimensions();
    const int total_size = dim.x() * dim.y() * dim.z();
    const int start_addr = toAddress(start_idx);
    const int goal_addr = toAddress(goal_idx);

    std::vector<NodeRecord> records(total_size);
    for (NodeRecord& record : records)
    {
        record.g = std::numeric_limits<double>::infinity();
        record.f = std::numeric_limits<double>::infinity();
        record.parent = -1;
        record.closed = false;
    }

    using OpenNode = std::pair<double, int>;
    std::priority_queue<OpenNode, std::vector<OpenNode>, std::greater<OpenNode>> open_set;

    records[start_addr].g = 0.0;
    records[start_addr].f = config_.heuristic_weight * heuristic(start_idx, goal_idx);
    records[start_addr].parent = start_addr;
    open_set.emplace(records[start_addr].f, start_addr);

    const std::vector<Eigen::Vector3i> offsets = neighborOffsets();
    int expanded_nodes = 0;
    last_expanded_nodes_ = 0;
    const ros::WallTime search_start = ros::WallTime::now();

    while (!open_set.empty() && expanded_nodes < config_.max_search_nodes)
    {
        if (config_.max_search_time > 1.0e-9 &&
            (ros::WallTime::now() - search_start).toSec() > config_.max_search_time)
        {
            last_error_ = "A* reached max_search_time.";
            return false;
        }

        if (cancel_callback_ && cancel_callback_())
        {
            last_error_ = "A* preempted by a new goal.";
            return false;
        }

        const int current_addr = open_set.top().second;
        open_set.pop();

        if (records[current_addr].closed)
        {
            continue;
        }

        records[current_addr].closed = true;
        ++expanded_nodes;
        last_expanded_nodes_ = expanded_nodes;

        const Eigen::Vector3i current_idx = addressToIndex(current_addr);
        searched_nodes_.push_back(map_->indexToPos(current_idx));

        if (current_addr == goal_addr)
        {
            std::vector<Eigen::Vector3d> reversed_path;
            int trace_addr = goal_addr;
            while (trace_addr != start_addr)
            {
                reversed_path.push_back(map_->indexToPos(addressToIndex(trace_addr)));
                trace_addr = records[trace_addr].parent;
                if (trace_addr < 0)
                {
                    last_error_ = "Broken A* parent chain.";
                    return false;
                }
            }
            reversed_path.push_back(map_->indexToPos(start_idx));

            path.assign(reversed_path.rbegin(), reversed_path.rend());
            if (!path.empty())
            {
                path.front() = start;
                path.back() = goal;
            }
            return true;
        }

        const Eigen::Vector3d current_pos = map_->indexToPos(current_idx);
        for (const Eigen::Vector3i& offset : offsets)
        {
            const Eigen::Vector3i neighbor_idx = current_idx + offset;
            if (!map_->isInMap(neighbor_idx))
            {
                continue;
            }

            const int neighbor_addr = toAddress(neighbor_idx);
            if (records[neighbor_addr].closed)
            {
                continue;
            }

            const Eigen::Vector3d neighbor_pos = map_->indexToPos(neighbor_idx);
            const bool neighbor_is_goal = (neighbor_addr == goal_addr);
            if (!neighbor_is_goal && map_->isInflatedOccupied(neighbor_pos))
            {
                continue;
            }

            const bool current_is_start = (current_addr == start_addr);
            if (config_.check_line_collision &&
                !current_is_start &&
                !neighbor_is_goal &&
                !map_->isLineFree(current_pos, neighbor_pos, config_.line_check_step))
            {
                continue;
            }

            const double edge_cost = offset.cast<double>().norm() * map_->resolution();
            const double tentative_g = records[current_addr].g + edge_cost;
            if (tentative_g >= records[neighbor_addr].g)
            {
                continue;
            }

            records[neighbor_addr].g = tentative_g;
            records[neighbor_addr].f = tentative_g + config_.heuristic_weight * heuristic(neighbor_idx, goal_idx);
            records[neighbor_addr].parent = current_addr;
            open_set.emplace(records[neighbor_addr].f, neighbor_addr);
        }
    }

    if (expanded_nodes >= config_.max_search_nodes)
    {
        last_error_ = "A* reached max_search_nodes.";
    }
    else
    {
        last_error_ = "A* failed to find a path.";
    }
    return false;
}

bool AStarPlanner::planToHorizon(const Eigen::Vector3d& start,
                                 const Eigen::Vector3d& final_goal,
                                 double horizon,
                                 std::vector<Eigen::Vector3d>& path)
{
    last_attempt_count_ = 0;
    last_clearance_used_ = config_.min_clearance;
    ++last_attempt_count_;

    if (planToHorizonOnFrontendMap(start, final_goal, horizon, path))
    {
        return true;
    }

    if (cancel_callback_ && cancel_callback_())
    {
        return false;
    }

    last_error_ = "Guide A* failed on current inflated map: " + last_error_;
    return false;
}

bool AStarPlanner::planToHorizonOnFrontendMap(const Eigen::Vector3d& start,
                                              const Eigen::Vector3d& final_goal,
                                              double horizon,
                                              std::vector<Eigen::Vector3d>& path)
{
    path.clear();
    searched_nodes_.clear();
    last_error_.clear();

    if (!map_)
    {
        last_error_ = "VoxelMap is not set.";
        return false;
    }

    Eigen::Vector3i start_idx;
    if (!map_->posToIndex(start, start_idx))
    {
        last_error_ = "Guide start is outside local map.";
        return false;
    }
    // 与普通 A* 一致，guide search 允许 start 位于 frontend 额外 clearance 内，
    // 第一段会跳过线段检查以便从保守膨胀区向外逃离。

    Eigen::Vector3i goal_idx;
    const bool goal_in_map = map_->posToIndex(final_goal, goal_idx);
    const double stop_horizon = std::max(map_->resolution(), horizon);

    const Eigen::Vector3i dim = map_->dimensions();
    const int total_size = dim.x() * dim.y() * dim.z();
    const int start_addr = toAddress(start_idx);
    const int goal_addr = goal_in_map ? toAddress(goal_idx) : -1;

    std::vector<NodeRecord> records(total_size);
    for (NodeRecord& record : records)
    {
        record.g = std::numeric_limits<double>::infinity();
        record.f = std::numeric_limits<double>::infinity();
        record.parent = -1;
        record.closed = false;
    }

    auto heuristic_to_goal = [this, &final_goal](const Eigen::Vector3i& idx) {
        return (map_->indexToPos(idx) - final_goal).norm();
    };

    using OpenNode = std::pair<double, int>;
    std::priority_queue<OpenNode, std::vector<OpenNode>, std::greater<OpenNode>> open_set;

    records[start_addr].g = 0.0;
    records[start_addr].f = config_.heuristic_weight * heuristic_to_goal(start_idx);
    records[start_addr].parent = start_addr;
    open_set.emplace(records[start_addr].f, start_addr);

    const std::vector<Eigen::Vector3i> offsets = neighborOffsets();
    int expanded_nodes = 0;
    last_expanded_nodes_ = 0;
    const ros::WallTime search_start = ros::WallTime::now();

    while (!open_set.empty() && expanded_nodes < config_.max_search_nodes)
    {
        if (config_.max_search_time > 1.0e-9 &&
            (ros::WallTime::now() - search_start).toSec() > config_.max_search_time)
        {
            last_error_ = "Guide A* reached max_search_time.";
            return false;
        }

        if (cancel_callback_ && cancel_callback_())
        {
            last_error_ = "Guide A* preempted by a new goal.";
            return false;
        }

        const int current_addr = open_set.top().second;
        open_set.pop();

        if (records[current_addr].closed)
        {
            continue;
        }

        records[current_addr].closed = true;
        ++expanded_nodes;
        last_expanded_nodes_ = expanded_nodes;

        const Eigen::Vector3i current_idx = addressToIndex(current_addr);
        searched_nodes_.push_back(map_->indexToPos(current_idx));

        // guide search 的停止条件有两个：
        // 1. 若最终目标在局部地图内且已到达，则直接返回到最终目标；
        // 2. 若累计路径长度 $g$ 已达到 planning horizon，则返回当前节点作为 local target。
        const bool reached_goal = goal_in_map && current_addr == goal_addr;
        const bool reached_horizon = current_addr != start_addr &&
                                     records[current_addr].g >= stop_horizon;
        if (reached_goal || reached_horizon)
        {
            std::vector<Eigen::Vector3d> reversed_path;
            int trace_addr = current_addr;
            while (trace_addr != start_addr)
            {
                reversed_path.push_back(map_->indexToPos(addressToIndex(trace_addr)));
                trace_addr = records[trace_addr].parent;
                if (trace_addr < 0)
                {
                    last_error_ = "Broken guide A* parent chain.";
                    return false;
                }
            }
            reversed_path.push_back(map_->indexToPos(start_idx));

            path.assign(reversed_path.rbegin(), reversed_path.rend());
            if (!path.empty())
            {
                path.front() = start;
                if (reached_goal)
                {
                    path.back() = final_goal;
                }
            }
            return path.size() >= 2;
        }

        const Eigen::Vector3d current_pos = map_->indexToPos(current_idx);
        for (const Eigen::Vector3i& offset : offsets)
        {
            const Eigen::Vector3i neighbor_idx = current_idx + offset;
            if (!map_->isInMap(neighbor_idx))
            {
                continue;
            }

            const int neighbor_addr = toAddress(neighbor_idx);
            if (records[neighbor_addr].closed)
            {
                continue;
            }

            const Eigen::Vector3d neighbor_pos = map_->indexToPos(neighbor_idx);
            const bool neighbor_is_goal = goal_in_map && neighbor_addr == goal_addr;
            if (!neighbor_is_goal && map_->isInflatedOccupied(neighbor_pos))
            {
                continue;
            }

            const bool current_is_start = (current_addr == start_addr);
            if (config_.check_line_collision &&
                !current_is_start &&
                !neighbor_is_goal &&
                !map_->isLineFree(current_pos, neighbor_pos, config_.line_check_step))
            {
                continue;
            }

            const double edge_cost = offset.cast<double>().norm() * map_->resolution();
            const double tentative_g = records[current_addr].g + edge_cost;
            if (tentative_g >= records[neighbor_addr].g)
            {
                continue;
            }

            records[neighbor_addr].g = tentative_g;
            records[neighbor_addr].f = tentative_g + config_.heuristic_weight * heuristic_to_goal(neighbor_idx);
            records[neighbor_addr].parent = current_addr;
            open_set.emplace(records[neighbor_addr].f, neighbor_addr);
        }
    }

    if (expanded_nodes >= config_.max_search_nodes)
    {
        last_error_ = "Guide A* reached max_search_nodes.";
    }
    else
    {
        last_error_ = "Guide A* failed before reaching horizon.";
    }
    return false;
}

const std::vector<Eigen::Vector3d>& AStarPlanner::searchedNodes() const
{
    return searched_nodes_;
}

std::string AStarPlanner::lastError() const
{
    return last_error_;
}

int AStarPlanner::toAddress(const Eigen::Vector3i& idx) const
{
    const Eigen::Vector3i dim = map_->dimensions();
    return idx.x() * dim.y() * dim.z() + idx.y() * dim.z() + idx.z();
}

Eigen::Vector3i AStarPlanner::addressToIndex(int address) const
{
    const Eigen::Vector3i dim = map_->dimensions();
    Eigen::Vector3i idx;
    idx.x() = address / (dim.y() * dim.z());
    const int rem = address % (dim.y() * dim.z());
    idx.y() = rem / dim.z();
    idx.z() = rem % dim.z();
    return idx;
}

double AStarPlanner::heuristic(const Eigen::Vector3i& current,
                               const Eigen::Vector3i& goal) const
{
    return (current - goal).cast<double>().norm() * map_->resolution();
}

std::vector<Eigen::Vector3i> AStarPlanner::neighborOffsets() const
{
    std::vector<Eigen::Vector3i> offsets;
    offsets.reserve(config_.allow_diagonal ? 26 : 6);

    for (int dx = -1; dx <= 1; ++dx)
    {
        for (int dy = -1; dy <= 1; ++dy)
        {
            for (int dz = -1; dz <= 1; ++dz)
            {
                if (dx == 0 && dy == 0 && dz == 0)
                {
                    continue;
                }

                const int manhattan = std::abs(dx) + std::abs(dy) + std::abs(dz);
                if (!config_.allow_diagonal && manhattan != 1)
                {
                    continue;
                }

                offsets.emplace_back(dx, dy, dz);
            }
        }
    }

    return offsets;
}

}  // namespace fastnav_planner
