#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>

#include <fastnav_mapping/voxel_map.h>

namespace fastnav_planner
{

// PathOptimizer 是 planner 内部的纯算法优化器类，不订阅 ROS topic，也不生成控制量。
// 当前第一版只对 A* 折线路径做 shortcut：若线段 $[p_i,p_j]$ 在 inflated map 中无碰撞，就删除中间冗余点。
class PathOptimizer
{
public:
    struct Config
    {
        bool enable{true};
        bool shortcut{true};
        double line_check_step{0.1};
    };

    void setConfig(const Config& config);

    bool optimize(const std::vector<Eigen::Vector3d>& raw_path,
                  const std::shared_ptr<fastnav_mapping::VoxelMap>& map,
                  std::vector<Eigen::Vector3d>& optimized_path);

    std::string lastError() const;

private:
    bool shortcutPath(const std::vector<Eigen::Vector3d>& raw_path,
                      const std::shared_ptr<fastnav_mapping::VoxelMap>& map,
                      std::vector<Eigen::Vector3d>& optimized_path) const;

private:
    Config config_;
    std::string last_error_;
};

}  // namespace fastnav_planner
