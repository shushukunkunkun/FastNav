#include "fastnav_planner/path_optimizer.h"

namespace fastnav_planner
{

void PathOptimizer::setConfig(const Config& config)
{
    config_ = config;
}

bool PathOptimizer::optimize(const std::vector<Eigen::Vector3d>& raw_path,
                             const std::shared_ptr<fastnav_mapping::VoxelMap>& map,
                             std::vector<Eigen::Vector3d>& optimized_path)
{
    last_error_.clear();
    optimized_path.clear();

    if (raw_path.empty())
    {
        last_error_ = "Raw path is empty.";
        return false;
    }

    if (!config_.enable)
    {
        optimized_path = raw_path;
        return true;
    }

    if (!map)
    {
        last_error_ = "VoxelMap is not set.";
        return false;
    }

    if (config_.shortcut)
    {
        return shortcutPath(raw_path, map, optimized_path);
    }

    optimized_path = raw_path;
    return true;
}

std::string PathOptimizer::lastError() const
{
    return last_error_;
}

bool PathOptimizer::shortcutPath(const std::vector<Eigen::Vector3d>& raw_path,
                                 const std::shared_ptr<fastnav_mapping::VoxelMap>& map,
                                 std::vector<Eigen::Vector3d>& optimized_path) const
{
    if (raw_path.size() <= 2)
    {
        optimized_path = raw_path;
        return true;
    }

    // 从当前点 $p_i$ 开始，寻找最远的 $p_j$，使线段 $p_i -> p_j$ 不穿过膨胀障碍。
    // 这样可以保留路径拓扑，同时删除 A* 栅格搜索带来的锯齿点。
    optimized_path.push_back(raw_path.front());
    size_t i = 0;
    while (i + 1 < raw_path.size())
    {
        size_t best = i + 1;
        for (size_t j = raw_path.size() - 1; j > i + 1; --j)
        {
            if (map->isLineFree(raw_path[i], raw_path[j], config_.line_check_step))
            {
                best = j;
                break;
            }
        }

        optimized_path.push_back(raw_path[best]);
        i = best;
    }

    return true;
}

}  // namespace fastnav_planner
