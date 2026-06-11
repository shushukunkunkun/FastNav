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
    config_.projection_margin_voxels = std::max(0, config_.projection_margin_voxels);
    config_.nearest_free_search_radius = std::max(0.0, config_.nearest_free_search_radius);
    config_.timeout_best_effort_min_length = std::max(0.0, config_.timeout_best_effort_min_length);
    config_.timeout_horizon_scale = std::min(0.95, std::max(0.05, config_.timeout_horizon_scale));
    config_.timeout_min_horizon = std::max(0.0, config_.timeout_min_horizon);
    config_.escape_max_radius = std::max(0.0, config_.escape_max_radius);
    config_.escape_max_search_time = std::max(0.0, config_.escape_max_search_time);
    config_.escape_max_nodes = std::max(1, config_.escape_max_nodes);
}

void AStarPlanner::setCancelCallback(const std::function<bool()>& cancel_callback)
{
    cancel_callback_ = cancel_callback;
}

bool AStarPlanner::isTimeOut(const ros::WallTime& search_start, double limit) const
{
    return limit > 1.0e-9 && (ros::WallTime::now() - search_start).toSec() > limit;
}

bool AStarPlanner::isPreempted() const
{
    return cancel_callback_ && cancel_callback_();
}

bool AStarPlanner::retrievePath(const std::vector<NodeRecord>& records,
                                int start_addr,
                                int end_addr,
                                const Eigen::Vector3d& start,
                                const Eigen::Vector3d& end,
                                std::vector<Eigen::Vector3d>& path) const
{
    std::vector<Eigen::Vector3d> reversed_path;
    int trace_addr = end_addr;
    while (trace_addr != start_addr)
    {
        reversed_path.push_back(map_->indexToPos(addressToIndex(trace_addr)));
        trace_addr = records[trace_addr].parent;
        if (trace_addr < 0)
        {
            return false;
        }
    }
    reversed_path.push_back(map_->indexToPos(addressToIndex(start_addr)));

    path.assign(reversed_path.rbegin(), reversed_path.rend());
    if (!path.empty())
    {
        path.front() = start;
        path.back() = end;
    }
    return path.size() >= 2;
}

Eigen::Vector3d AStarPlanner::clampInsideMap(const Eigen::Vector3d& pos, double margin) const
{
    const Eigen::Vector3d box_min = map_->getOrigin() + Eigen::Vector3d::Constant(margin);
    const Eigen::Vector3d box_max = map_->getCorner() - Eigen::Vector3d::Constant(margin);
    if ((box_max.array() <= box_min.array()).any())
    {
        return pos.cwiseMax(map_->getOrigin()).cwiseMin(map_->getCorner() - Eigen::Vector3d::Constant(1.0e-6));
    }
    return pos.cwiseMax(box_min).cwiseMin(box_max);
}

bool AStarPlanner::projectGoalIntoMap(const Eigen::Vector3d& start,
                                      const Eigen::Vector3d& goal,
                                      Eigen::Vector3d& projected_goal,
                                      bool& was_projected) const
{
    was_projected = false;
    projected_goal = goal;
    Eigen::Vector3i goal_idx;
    if (map_->posToIndex(goal, goal_idx))
    {
        return true;
    }
    if (!config_.enable_goal_projection)
    {
        return false;
    }

    const Eigen::Vector3d dir = goal - start;
    const double dir_norm = dir.norm();
    if (dir_norm < 1.0e-6)
    {
        return false;
    }

    const Eigen::Vector3d box_min = map_->getOrigin();
    const Eigen::Vector3d box_max = map_->getCorner();
    double t_exit = 1.0;
    bool found_exit = false;

    // 若终点在局部地图外，求线段 $p(s)=p_0+s(p_g-p_0)$ 与地图 AABB 的出界参数 $s^\*$。
    // 对每个维度独立求交，取最小正参数，即 $s^\*=\min_i ((b_i-p_{0,i})/d_i)$。
    for (int axis = 0; axis < 3; ++axis)
    {
        if (dir[axis] > 1.0e-9)
        {
            const double t = (box_max[axis] - start[axis]) / dir[axis];
            if (t >= 0.0)
            {
                t_exit = std::min(t_exit, t);
                found_exit = true;
            }
        }
        else if (dir[axis] < -1.0e-9)
        {
            const double t = (box_min[axis] - start[axis]) / dir[axis];
            if (t >= 0.0)
            {
                t_exit = std::min(t_exit, t);
                found_exit = true;
            }
        }
    }

    if (!found_exit)
    {
        projected_goal = clampInsideMap(goal, map_->resolution());
    }
    else
    {
        const double margin = std::max(1.0, static_cast<double>(config_.projection_margin_voxels)) * map_->resolution();
        const Eigen::Vector3d hit_point = start + std::max(0.0, std::min(1.0, t_exit)) * dir;
        // 从边界点沿单位方向 $\hat d=(p_g-p_0)/\|p_g-p_0\|$ 退回 $m$，得到地图内部投影点。
        projected_goal = clampInsideMap(hit_point - margin * dir / dir_norm, margin * 0.5);
    }

    was_projected = true;
    return map_->isInMap(projected_goal);
}

bool AStarPlanner::findNearestFreePosition(const Eigen::Vector3d& seed,
                                           double search_radius,
                                           Eigen::Vector3d& free_pos) const
{
    Eigen::Vector3i seed_idx;
    const Eigen::Vector3d clamped_seed = clampInsideMap(seed, map_->resolution() * 0.5);
    if (!map_->posToIndex(clamped_seed, seed_idx))
    {
        return false;
    }

    const int max_step = std::max(0, static_cast<int>(std::ceil(search_radius / map_->resolution())));
    double best_dist = std::numeric_limits<double>::infinity();
    bool found = false;
    Eigen::Vector3d best_pos = clamped_seed;

    // 在以 seed 为中心的体素球内寻找最近 free 点，目标是最小化 $\|p_i-p_s\|$。
    // 这里搜索的是当前 map 的 inflated occupancy，因此返回点满足 $p_i \notin \mathcal{O}_{inf}$。
    for (int dx = -max_step; dx <= max_step; ++dx)
    {
        for (int dy = -max_step; dy <= max_step; ++dy)
        {
            for (int dz = -max_step; dz <= max_step; ++dz)
            {
                const Eigen::Vector3i idx = seed_idx + Eigen::Vector3i(dx, dy, dz);
                if (!map_->isInMap(idx))
                {
                    continue;
                }

                const Eigen::Vector3d pos = map_->indexToPos(idx);
                const double dist = (pos - seed).norm();
                if (dist > search_radius + 1.0e-6 || dist >= best_dist)
                {
                    continue;
                }
                if (map_->isInflatedOccupied(pos))
                {
                    continue;
                }

                found = true;
                best_dist = dist;
                best_pos = pos;
            }
        }
    }

    if (!found)
    {
        return false;
    }
    free_pos = best_pos;
    return true;
}

bool AStarPlanner::plan(const Eigen::Vector3d& start,
                        const Eigen::Vector3d& goal,
                        std::vector<Eigen::Vector3d>& path)
{
    last_attempt_count_ = 0;
    last_clearance_used_ = config_.min_clearance;
    last_status_ = SearchStatus::NO_PATH;
    last_path_target_ = goal;
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
        last_status_ = SearchStatus::INIT_ERROR;
        return false;
    }

    Eigen::Vector3d search_goal;
    bool goal_projected = false;
    if (!projectGoalIntoMap(start, goal, search_goal, goal_projected))
    {
        last_error_ = "Goal is outside local map.";
        last_status_ = SearchStatus::INIT_ERROR;
        return false;
    }

    if (map_->isInflatedOccupied(search_goal))
    {
        Eigen::Vector3d free_goal;
        if (!findNearestFreePosition(search_goal, config_.nearest_free_search_radius, free_goal))
        {
            last_error_ = "Goal is inside inflated obstacle.";
            last_status_ = SearchStatus::INIT_ERROR;
            return false;
        }
        search_goal = free_goal;
        goal_projected = true;
    }

    if (!map_->posToIndex(search_goal, goal_idx))
    {
        last_error_ = "Projected goal is outside local map.";
        last_status_ = SearchStatus::INIT_ERROR;
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
    int best_addr = start_addr;
    double best_score = std::numeric_limits<double>::infinity();

    while (!open_set.empty() && expanded_nodes < config_.max_search_nodes)
    {
        if (isTimeOut(search_start, config_.max_search_time))
        {
            if (config_.allow_timeout_best_effort &&
                best_addr != start_addr &&
                records[best_addr].g >= config_.timeout_best_effort_min_length &&
                retrievePath(records, start_addr, best_addr, start, map_->indexToPos(addressToIndex(best_addr)), path))
            {
                last_path_target_ = path.back();
                last_status_ = SearchStatus::BEST_EFFORT;
                last_error_ = "A* reached max_search_time, return best-effort path.";
                return true;
            }
            last_error_ = "A* reached max_search_time.";
            last_status_ = SearchStatus::TIME_OUT;
            return false;
        }

        if (isPreempted())
        {
            last_error_ = "A* preempted by a new goal.";
            last_status_ = SearchStatus::PREEMPTED;
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
        if (current_addr != start_addr)
        {
            const double h = (map_->indexToPos(current_idx) - search_goal).norm();
            // best-effort 选择启发值 $h(p)=\|p-p_g\|$ 最小的已展开节点；
            // 若 A* 超时，返回 $start \rightarrow p_{best}$，保证前端至少输出一段可执行推进方向。
            if (h < best_score)
            {
                best_score = h;
                best_addr = current_addr;
            }
        }

        if (current_addr == goal_addr)
        {
            if (!retrievePath(records, start_addr, goal_addr, start, search_goal, path))
            {
                last_error_ = "Broken A* parent chain.";
                last_status_ = SearchStatus::NO_PATH;
                return false;
            }
            last_path_target_ = search_goal;
            last_status_ = goal_projected ? SearchStatus::BEST_EFFORT : SearchStatus::REACH_GOAL;
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
        if (config_.allow_timeout_best_effort &&
            best_addr != start_addr &&
            records[best_addr].g >= config_.timeout_best_effort_min_length &&
            retrievePath(records, start_addr, best_addr, start, map_->indexToPos(addressToIndex(best_addr)), path))
        {
            last_path_target_ = path.back();
            last_status_ = SearchStatus::BEST_EFFORT;
            last_error_ = "A* reached max_search_nodes, return best-effort path.";
            return true;
        }
        last_error_ = "A* reached max_search_nodes.";
        last_status_ = SearchStatus::TIME_OUT;
    }
    else
    {
        last_error_ = "A* failed to find a path.";
        last_status_ = SearchStatus::NO_PATH;
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
    last_status_ = SearchStatus::NO_PATH;
    last_path_target_ = final_goal;
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
        last_status_ = SearchStatus::INIT_ERROR;
        return false;
    }
    // 与普通 A* 一致，guide search 允许 start 位于 frontend 额外 clearance 内，
    // 第一段会跳过线段检查以便从保守膨胀区向外逃离。

    Eigen::Vector3i goal_idx;
    Eigen::Vector3d search_goal = final_goal;
    bool goal_in_map = map_->posToIndex(search_goal, goal_idx);
    if (goal_in_map && map_->isInflatedOccupied(search_goal))
    {
        Eigen::Vector3d free_goal;
        if (findNearestFreePosition(search_goal, config_.nearest_free_search_radius, free_goal))
        {
            search_goal = free_goal;
            goal_in_map = map_->posToIndex(search_goal, goal_idx);
        }
    }
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

    auto heuristic_to_goal = [this, &search_goal](const Eigen::Vector3i& idx) {
        return (map_->indexToPos(idx) - search_goal).norm();
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
    int best_addr = start_addr;
    double best_score = std::numeric_limits<double>::infinity();

    while (!open_set.empty() && expanded_nodes < config_.max_search_nodes)
    {
        if (isTimeOut(search_start, config_.max_search_time))
        {
            if (config_.allow_timeout_best_effort &&
                best_addr != start_addr &&
                records[best_addr].g >= config_.timeout_best_effort_min_length &&
                retrievePath(records, start_addr, best_addr, start, map_->indexToPos(addressToIndex(best_addr)), path))
            {
                last_path_target_ = path.back();
                last_status_ = SearchStatus::BEST_EFFORT;
                last_error_ = "Guide A* reached max_search_time, return best-effort path.";
                return true;
            }
            last_error_ = "Guide A* reached max_search_time.";
            last_status_ = SearchStatus::TIME_OUT;
            return false;
        }

        if (isPreempted())
        {
            last_error_ = "Guide A* preempted by a new goal.";
            last_status_ = SearchStatus::PREEMPTED;
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
        if (current_addr != start_addr)
        {
            const double h = heuristic_to_goal(current_idx);
            if (h < best_score)
            {
                best_score = h;
                best_addr = current_addr;
            }
        }

        // guide search 的停止条件有两个：
        // 1. 若最终目标在局部地图内且已到达，则直接返回到最终目标；
        // 2. 若累计路径长度 $g$ 已达到 planning horizon，则返回当前节点作为 local target。
        const bool reached_goal = goal_in_map && current_addr == goal_addr;
        const bool reached_horizon = current_addr != start_addr &&
                                     records[current_addr].g >= stop_horizon;
        if (reached_goal || reached_horizon)
        {
            const Eigen::Vector3d path_end = reached_goal ? search_goal : map_->indexToPos(current_idx);
            if (!retrievePath(records, start_addr, current_addr, start, path_end, path))
            {
                last_error_ = "Broken guide A* parent chain.";
                last_status_ = SearchStatus::NO_PATH;
                return false;
            }
            last_path_target_ = path.back();
            last_status_ = reached_goal ? SearchStatus::REACH_GOAL : SearchStatus::REACH_HORIZON;
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
        if (config_.allow_timeout_best_effort &&
            best_addr != start_addr &&
            records[best_addr].g >= config_.timeout_best_effort_min_length &&
            retrievePath(records, start_addr, best_addr, start, map_->indexToPos(addressToIndex(best_addr)), path))
        {
            last_path_target_ = path.back();
            last_status_ = SearchStatus::BEST_EFFORT;
            last_error_ = "Guide A* reached max_search_nodes, return best-effort path.";
            return true;
        }
        last_error_ = "Guide A* reached max_search_nodes.";
        last_status_ = SearchStatus::TIME_OUT;
    }
    else
    {
        last_error_ = "Guide A* failed before reaching horizon.";
        last_status_ = SearchStatus::NO_PATH;
    }
    return false;
}

bool AStarPlanner::escapeFromInflatedRegion(
    const Eigen::Vector3d& start,
    const std::shared_ptr<fastnav_mapping::VoxelMap>& blocking_map,
    std::vector<Eigen::Vector3d>& path)
{
    path.clear();
    searched_nodes_.clear();
    last_error_.clear();
    last_status_ = SearchStatus::NO_PATH;
    last_path_target_ = start;
    last_expanded_nodes_ = 0;

    if (!map_ || !blocking_map)
    {
        last_error_ = "Escape search maps are not ready.";
        last_status_ = SearchStatus::INIT_ERROR;
        return false;
    }

    Eigen::Vector3i start_idx;
    if (!map_->posToIndex(start, start_idx))
    {
        last_error_ = "Escape start is outside base map.";
        last_status_ = SearchStatus::INIT_ERROR;
        return false;
    }

    if (!blocking_map->isInflatedOccupied(start))
    {
        path.push_back(start);
        last_path_target_ = start;
        last_status_ = SearchStatus::SUCCESS;
        return true;
    }

    const Eigen::Vector3i dim = map_->dimensions();
    const int total_size = dim.x() * dim.y() * dim.z();
    const int start_addr = toAddress(start_idx);

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
    records[start_addr].f = 0.0;
    records[start_addr].parent = start_addr;
    open_set.emplace(0.0, start_addr);

    const std::vector<Eigen::Vector3i> offsets = neighborOffsets();
    const ros::WallTime search_start = ros::WallTime::now();
    int expanded_nodes = 0;

    while (!open_set.empty() && expanded_nodes < config_.escape_max_nodes)
    {
        if (isTimeOut(search_start, config_.escape_max_search_time))
        {
            last_error_ = "Escape search reached max_search_time.";
            last_status_ = SearchStatus::TIME_OUT;
            return false;
        }
        if (isPreempted())
        {
            last_error_ = "Escape search preempted by a new goal.";
            last_status_ = SearchStatus::PREEMPTED;
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
        const Eigen::Vector3d current_pos = map_->indexToPos(current_idx);
        searched_nodes_.push_back(current_pos);

        // Escape 的终止条件是离开 frontend 膨胀障碍集合：
        // $p \notin \mathcal{O}_{front}$。搜索通行性仍由 base map 约束，避免穿过真实障碍。
        if (current_addr != start_addr && !blocking_map->isInflatedOccupied(current_pos))
        {
            if (!retrievePath(records, start_addr, current_addr, start, current_pos, path))
            {
                last_error_ = "Broken escape A* parent chain.";
                last_status_ = SearchStatus::NO_PATH;
                return false;
            }
            last_path_target_ = current_pos;
            last_status_ = SearchStatus::SUCCESS;
            return true;
        }

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
            const double dist_from_start = (neighbor_pos - start).norm();
            if (dist_from_start > config_.escape_max_radius + 1.0e-6)
            {
                continue;
            }
            if (map_->isInflatedOccupied(neighbor_pos))
            {
                continue;
            }

            const bool current_is_start = (current_addr == start_addr);
            if (config_.check_line_collision &&
                !current_is_start &&
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

            // Escape 不追求某个固定终点，因此代价只使用 Dijkstra 的 $g$。
            records[neighbor_addr].g = tentative_g;
            records[neighbor_addr].f = tentative_g;
            records[neighbor_addr].parent = current_addr;
            open_set.emplace(records[neighbor_addr].f, neighbor_addr);
        }
    }

    last_expanded_nodes_ = expanded_nodes;
    last_error_ = expanded_nodes >= config_.escape_max_nodes
                      ? "Escape search reached max_search_nodes."
                      : "Escape search failed to leave frontend inflated region.";
    last_status_ = expanded_nodes >= config_.escape_max_nodes ? SearchStatus::TIME_OUT : SearchStatus::NO_PATH;
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
