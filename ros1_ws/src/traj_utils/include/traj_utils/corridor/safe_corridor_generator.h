#pragma once

#include <string>
#include <vector>

#include <Eigen/Eigen>

namespace traj_utils
{

// SafeCorridorGenerator 是通用轨迹工具，不直接依赖 FastNav 的 VoxelMap。
// 调用方只需要提供几何路径 $r_k$、障碍表面点 $p_j$ 和地图边界，就能生成凸走廊 $\mathcal{C}_i=\{x\mid H_i[x^T,1]^T\le0\}$。
class SafeCorridorGenerator
{
public:
    struct Config
    {
        // 每个走廊段沿路径最多覆盖 progress 米，较小会生成更多段，较大则单段 FIRI 计算更重。
        double progress{3.0};
        // 每段路径周围取 range 米局部盒，只把盒内障碍点交给 FIRI。
        double range{3.0};
        // 判断相邻走廊是否需要补 gap polytope 的数值阈值。
        double eps{1.0e-6};
        // 是否删除冗余走廊段；若 $\mathcal{P}_i\cap\mathcal{P}_j\ne\emptyset$，中间段可以被跳过。
        bool enable_shortcut{true};
    };

    void setConfig(const Config& config);

    // 输入 path、obstacle_surface_points、地图下界 low_corner 和上界 high_corner，输出 H-polytope 序列。
    // 每个 hpoly 的一行表示 $h_0x+h_1y+h_2z+h_3\le0$。
    bool generate(const std::vector<Eigen::Vector3d>& path,
                  const std::vector<Eigen::Vector3d>& obstacle_surface_points,
                  const Eigen::Vector3d& low_corner,
                  const Eigen::Vector3d& high_corner,
                  std::vector<Eigen::MatrixX4d>& hpolys);

    std::string lastError() const;

private:
    bool convexCover(const std::vector<Eigen::Vector3d>& path,
                     const std::vector<Eigen::Vector3d>& obstacle_surface_points,
                     const Eigen::Vector3d& low_corner,
                     const Eigen::Vector3d& high_corner,
                     std::vector<Eigen::MatrixX4d>& hpolys);

    void shortCut(std::vector<Eigen::MatrixX4d>& hpolys);

private:
    Config config_;
    std::string last_error_;
};

}  // namespace traj_utils
