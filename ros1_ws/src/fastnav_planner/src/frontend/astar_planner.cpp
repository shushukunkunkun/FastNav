#include "fastnav_planner/frontend/astar_planner.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>

namespace fastnav_planner
{

void AStarPlanner::setMap(const std::shared_ptr<fastnav_mapping::VoxelMap>& map)
{
    map_ = map;
}

void AStarPlanner::setConfig(const Config& config)
{
    config_ = config;
}

void AStarPlanner::setCancelCallback(const std::function<bool()>& cancel_callback)
{
    cancel_callback_ = cancel_callback;
}

bool AStarPlanner::plan(const Eigen::Vector3d& start,
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

    if (map_->isInflatedOccupied(start))
    {
        last_error_ = "Start is inside inflated obstacle.";
        return false;
    }
    if (map_->isInflatedOccupied(goal))
    {
        last_error_ = "Goal is inside inflated obstacle.";
        return false;
    }

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

    while (!open_set.empty() && expanded_nodes < config_.max_search_nodes)
    {
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
            if (map_->isInflatedOccupied(neighbor_pos))
            {
                continue;
            }

            const bool neighbor_is_goal = (neighbor_addr == goal_addr);
            const bool current_is_start = (current_addr == start_addr);

            // min_clearance 是给后端 corridor 预留空间的“中间路径”要求。
            // 起点和终点只要求不在 inflated obstacle 中：起点可能已经贴近障碍，需要允许它先离开；
            // 终点可能由用户设在低 clearance 但仍可到达的位置，不能因为额外余量导致 near-goal 后永远无法收敛。
            if (!neighbor_is_goal && !hasExtraClearance(neighbor_pos))
            {
                continue;
            }

            if (config_.check_line_collision &&
                !isSegmentClearWithExtraClearance(current_pos,
                                                  neighbor_pos,
                                                  current_is_start,
                                                  neighbor_is_goal))
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

bool AStarPlanner::hasExtraClearance(const Eigen::Vector3d& pos) const
{
    if (config_.min_clearance <= 1.0e-6)
    {
        return true;
    }

    Eigen::Vector3i center_idx;
    if (!map_->posToIndex(pos, center_idx))
    {
        return false;
    }

    // VoxelMap 按体素中心保存 inflated 状态。这里用半个体素对角线近似体素半径，
    // 将“点到 inflated 体素体积”的距离近似为 $||p-c_i|| - \sqrt{3}r/2$。
    const double resolution = map_->resolution();
    const double voxel_radius = 0.5 * std::sqrt(3.0) * resolution;
    const double check_radius = config_.min_clearance + voxel_radius;
    const int radius_step = static_cast<int>(std::ceil(check_radius / resolution));

    for (int dx = -radius_step; dx <= radius_step; ++dx)
    {
        for (int dy = -radius_step; dy <= radius_step; ++dy)
        {
            for (int dz = -radius_step; dz <= radius_step; ++dz)
            {
                const Eigen::Vector3i idx = center_idx + Eigen::Vector3i(dx, dy, dz);
                if (!map_->isInMap(idx))
                {
                    return false;
                }

                const Eigen::Vector3d voxel_center = map_->indexToPos(idx);
                if ((voxel_center - pos).norm() > check_radius + 1.0e-6)
                {
                    continue;
                }

                if (map_->isInflatedOccupied(voxel_center))
                {
                    return false;
                }
            }
        }
    }

    return true;
}

bool AStarPlanner::isSegmentClearWithExtraClearance(const Eigen::Vector3d& p0,
                                                    const Eigen::Vector3d& p1,
                                                    bool relax_start_clearance,
                                                    bool relax_goal_clearance) const
{
    if (!map_->isLineFree(p0, p1, config_.line_check_step))
    {
        return false;
    }
    if (config_.min_clearance <= 1.0e-6)
    {
        return true;
    }

    const double step = config_.line_check_step > 1.0e-6
                            ? config_.line_check_step
                            : map_->resolution() * 0.5;
    const double length = (p1 - p0).norm();
    const int sample_num = std::max(1, static_cast<int>(std::ceil(length / step)));

    for (int i = 0; i <= sample_num; ++i)
    {
        if ((relax_start_clearance && i == 0) ||
            (relax_goal_clearance && i == sample_num))
        {
            continue;
        }

        const double alpha = static_cast<double>(i) / static_cast<double>(sample_num);
        const Eigen::Vector3d p = p0 + alpha * (p1 - p0);
        if (!hasExtraClearance(p))
        {
            return false;
        }
    }

    return true;
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
