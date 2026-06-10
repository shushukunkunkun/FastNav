#pragma once

#include <cstdint>
#include <vector>

#include <Eigen/Core>

namespace fastnav_mapping
{

// VoxelMap 是 FastNav 的第一版局部体素地图，内部只使用 odom 坐标系。
// 它不关心点云来自 GZ 还是真机，只接受 odom 下的点 $p$ 并映射到体素索引 $i$。
class VoxelMap
{
public:
    static constexpr int8_t FREE = 0;
    static constexpr int8_t OCCUPIED = 1;
    static constexpr int8_t INFLATED = 2;

    VoxelMap() = default;

    // 初始化地图参数。局部地图尺寸固定，但地图中心会随无人机位置 $c$ 更新。
    void init(double resolution,
              double local_x_size,
              double local_y_size,
              double local_z_size,
              double local_z_min,
              double local_z_max,
              double drone_radius,
              double safety_margin);

    // 设置多帧 occupancy 缓存的 log-odds 参数，后续 planner 内部地图会用 $l = log(p/(1-p))$ 累积观测。
    void setOccupancyUpdateParams(double p_hit,
                                  double p_miss,
                                  double p_min,
                                  double p_max,
                                  double p_occ,
                                  double temporal_decay_log);

    // 设置局部地图中心 $c$。地图原点为 $origin = [c_x - L_x/2, c_y - L_y/2, c_z + z_min]^T$。
    void setMapCenter(const Eigen::Vector3d& center);

    // 清空地图，所有体素重新置为 $0$，即 free / unknown。
    void reset();

    // 将 occupancy log-odds 缓慢拉回未知值 $0$，用于遗忘没有被连续观测到的旧障碍。
    void decayOccupancy();

    // odom 坐标转体素索引，数学关系为 $i = floor((p - origin) / resolution)$。
    bool posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& idx) const;

    // 体素索引转 odom 坐标，返回体素中心 $p = origin + (i + 0.5) resolution$。
    Eigen::Vector3d indexToPos(const Eigen::Vector3i& idx) const;

    bool isInMap(const Eigen::Vector3i& idx) const;
    bool isInMap(const Eigen::Vector3d& pos) const;

    // 将点 $p$ 所在体素设置为 occupied，即状态 $1$。
    void setOccupied(const Eigen::Vector3d& pos);

    // 对 occupied 体素做安全膨胀，将半径 $inflation_radius$ 内的邻域体素设置为 inflated。
    void inflateObstacles();

    bool isOccupied(const Eigen::Vector3d& pos) const;
    bool isInflatedOccupied(const Eigen::Vector3d& pos) const;

    // 线段离散碰撞检测：沿 $p(t) = p0 + t(p1-p0)$ 采样，若任一点落入 inflated 体素则返回 false。
    bool isLineFree(const Eigen::Vector3d& p0,
                    const Eigen::Vector3d& p1,
                    double step_size) const;

    std::vector<Eigen::Vector3d> getOccupiedVoxelCenters() const;
    std::vector<Eigen::Vector3d> getInflatedVoxelCenters(bool include_occupied) const;

    // GCOPTER/MINCO safe corridor 兼容接口：后续 sfc_gen 可直接读取地图范围和分辨率。
    // getCorner() 返回局部地图最大角点 $corner = origin + dim * resolution$。
    Eigen::Vector3i getSize() const { return dim_; }
    double getScale() const { return resolution_; }
    Eigen::Vector3d getOrigin() const { return origin_; }
    Eigen::Vector3d getCorner() const { return origin_ + dim_.cast<double>() * resolution_; }

    // GCOPTER 风格碰撞查询：返回 false 表示 free，返回 true 表示 occupied / inflated / out of map。
    // 因此 sfc_gen 中的 $query(p)==0$ 仍然代表点 $p$ 可通行。
    bool query(const Eigen::Vector3d& pos) const;

    // 输出膨胀障碍的表面体素中心点。FIRI 用这些点构造分离平面 $n^T x + d \le 0$。
    // points 采用追加语义，调用方如果需要空集合应先 clear()。
    void getSurf(std::vector<Eigen::Vector3d>& points,
                 bool include_occupied = true) const;

    // 只输出局部盒 $[box_min, box_max]$ 内的表面体素。
    // safe corridor 只关心路径附近障碍，因此该接口避免每次 FIRI 都扫描整张局部地图。
    void getSurfInBox(const Eigen::Vector3d& box_min,
                      const Eigen::Vector3d& box_max,
                      std::vector<Eigen::Vector3d>& points,
                      bool include_occupied = true) const;

    const Eigen::Vector3i& dimensions() const { return dim_; }
    const Eigen::Vector3d& origin() const { return origin_; }
    double resolution() const { return resolution_; }
    double inflationRadius() const { return inflation_radius_; }

private:
    int toAddress(const Eigen::Vector3i& idx) const;
    Eigen::Vector3i addressToIndex(int address) const;
    bool isLogOccupied(int address) const;
    bool isBlockedAddress(int address, bool include_occupied) const;
    bool isSurfaceIndex(const Eigen::Vector3i& idx, bool include_occupied) const;
    void shiftBuffersForNewOrigin(const Eigen::Vector3d& new_origin);

private:
    bool initialized_{false};

    double resolution_{0.2};
    double inv_resolution_{5.0};
    double local_x_size_{20.0};
    double local_y_size_{20.0};
    double local_z_size_{6.0};
    double local_z_min_{-2.0};
    double local_z_max_{4.0};
    double drone_radius_{0.35};
    double safety_margin_{0.15};
    double inflation_radius_{0.5};

    double prob_hit_log_{0.8472978604};
    double prob_miss_log_{-0.2006706955};
    double clamp_min_log_{-1.9924301647};
    double clamp_max_log_{3.4760986898};
    double min_occupancy_log_{0.6190392084};
    double unknown_log_{0.0};
    double temporal_decay_log_{0.05};

    Eigen::Vector3i dim_{0, 0, 0};
    Eigen::Vector3d center_{0.0, 0.0, 0.0};
    Eigen::Vector3d origin_{0.0, 0.0, 0.0};
    bool origin_initialized_{false};

    std::vector<double> occupancy_log_buffer_;
    std::vector<int8_t> inflated_buffer_;
};

}  // namespace fastnav_mapping
