#include "fastnav_mapping/voxel_map.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace fastnav_mapping
{

// C++14 下 static constexpr 成员若被 std::fill/assign 等以引用形式 odr-use，需要在类外提供定义。
constexpr int8_t VoxelMap::FREE;
constexpr int8_t VoxelMap::OCCUPIED;
constexpr int8_t VoxelMap::INFLATED;

namespace
{
double clampProbability(double p)
{
    return std::min(0.999, std::max(0.001, p));
}

double logit(double p)
{
    const double q = clampProbability(p);
    return std::log(q / (1.0 - q));
}
}  // namespace

void VoxelMap::init(double resolution,
                    double local_x_size,
                    double local_y_size,
                    double local_z_size,
                    double local_z_min,
                    double local_z_max,
                    double drone_radius,
                    double safety_margin)
{
    if (resolution <= 0.0)
    {
        throw std::invalid_argument("VoxelMap resolution must be positive.");
    }
    if (local_x_size <= 0.0 || local_y_size <= 0.0 || local_z_size <= 0.0)
    {
        throw std::invalid_argument("VoxelMap local map size must be positive.");
    }

    resolution_ = resolution;
    inv_resolution_ = 1.0 / resolution_;
    local_x_size_ = local_x_size;
    local_y_size_ = local_y_size;
    local_z_min_ = local_z_min;
    local_z_max_ = local_z_max;

    // z 方向优先使用相对无人机高度的上下界，满足 $z in [c_z + z_min, c_z + z_max]$。
    const double z_span = local_z_max_ - local_z_min_;
    local_z_size_ = z_span > 0.0 ? z_span : local_z_size;

    drone_radius_ = std::max(0.0, drone_radius);
    safety_margin_ = std::max(0.0, safety_margin);
    inflation_radius_ = drone_radius_ + safety_margin_;

    // 维度由 $dim = ceil(size / resolution)$ 得到，保证局部范围至少被完整覆盖。
    dim_.x() = std::max(1, static_cast<int>(std::ceil(local_x_size_ * inv_resolution_)));
    dim_.y() = std::max(1, static_cast<int>(std::ceil(local_y_size_ * inv_resolution_)));
    dim_.z() = std::max(1, static_cast<int>(std::ceil(local_z_size_ * inv_resolution_)));

    occupancy_log_buffer_.assign(dim_.x() * dim_.y() * dim_.z(), unknown_log_);
    inflated_buffer_.assign(dim_.x() * dim_.y() * dim_.z(), FREE);
    initialized_ = true;
    setMapCenter(center_);
}

void VoxelMap::setOccupancyUpdateParams(double p_hit,
                                        double p_miss,
                                        double p_min,
                                        double p_max,
                                        double p_occ,
                                        double temporal_decay_log)
{
    prob_hit_log_ = logit(p_hit);
    prob_miss_log_ = logit(p_miss);
    clamp_min_log_ = logit(p_min);
    clamp_max_log_ = logit(p_max);
    min_occupancy_log_ = logit(p_occ);
    temporal_decay_log_ = std::max(0.0, temporal_decay_log);
}

void VoxelMap::setMapCenter(const Eigen::Vector3d& center)
{
    center_ = center;
    // 局部地图水平居中于无人机，z 方向使用相对高度范围；同时把原点吸附到体素网格。
    // 若直接令 $origin = c - L/2$，无人机悬停时厘米级 odom 抖动会让所有体素中心一起滑动，RViz 中看起来像闪烁。
    // 这里使用 $origin = floor(origin_raw / resolution) resolution$，让网格只在跨过一个体素边界时整体移动。
    const Eigen::Vector3d raw_origin(center_.x() - local_x_size_ * 0.5,
                                     center_.y() - local_y_size_ * 0.5,
                                     center_.z() + local_z_min_);

    Eigen::Vector3d new_origin;
    new_origin.x() = std::floor(raw_origin.x() * inv_resolution_) * resolution_;
    new_origin.y() = std::floor(raw_origin.y() * inv_resolution_) * resolution_;
    new_origin.z() = std::floor(raw_origin.z() * inv_resolution_) * resolution_;

    shiftBuffersForNewOrigin(new_origin);
}

void VoxelMap::reset()
{
    std::fill(occupancy_log_buffer_.begin(), occupancy_log_buffer_.end(), unknown_log_);
    std::fill(inflated_buffer_.begin(), inflated_buffer_.end(), FREE);
}

void VoxelMap::decayOccupancy()
{
    if (!initialized_ || temporal_decay_log_ <= 1e-9)
    {
        return;
    }

    // 没有 raycasting free-space 时，用温和时间衰减把旧观测拉回未知值 $l=0$，避免旧障碍永久残留。
    for (double& log_occ : occupancy_log_buffer_)
    {
        if (log_occ > unknown_log_)
        {
            log_occ = std::max(unknown_log_, log_occ - temporal_decay_log_);
        }
        else if (log_occ < unknown_log_)
        {
            log_occ = std::min(unknown_log_, log_occ + temporal_decay_log_);
        }
    }
}

bool VoxelMap::posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& idx) const
{
    if (!initialized_)
    {
        return false;
    }

    // 将 odom 坐标 $p$ 减去地图原点，再除以分辨率得到连续体素坐标，最后向下取整得到整数索引。
    const Eigen::Vector3d relative = (pos - origin_) * inv_resolution_;
    idx.x() = static_cast<int>(std::floor(relative.x()));
    idx.y() = static_cast<int>(std::floor(relative.y()));
    idx.z() = static_cast<int>(std::floor(relative.z()));

    return isInMap(idx);
}

Eigen::Vector3d VoxelMap::indexToPos(const Eigen::Vector3i& idx) const
{
    // 返回体素中心位置而不是最小角点，因此每一维都加 $0.5 resolution$。
    return origin_ + (idx.cast<double>() + Eigen::Vector3d::Constant(0.5)) * resolution_;
}

bool VoxelMap::isInMap(const Eigen::Vector3i& idx) const
{
    return idx.x() >= 0 && idx.x() < dim_.x() &&
           idx.y() >= 0 && idx.y() < dim_.y() &&
           idx.z() >= 0 && idx.z() < dim_.z();
}

bool VoxelMap::isInMap(const Eigen::Vector3d& pos) const
{
    Eigen::Vector3i idx;
    return posToIndex(pos, idx);
}

void VoxelMap::setOccupied(const Eigen::Vector3d& pos)
{
    Eigen::Vector3i idx;
    if (!posToIndex(pos, idx))
    {
        return;
    }

    // 点云命中执行 log-odds hit 更新：$l_i = clamp(l_i + l_{hit}, l_{min}, l_{max})$。
    const int address = toAddress(idx);
    occupancy_log_buffer_[address] = std::min(clamp_max_log_,
                                              std::max(clamp_min_log_,
                                                       occupancy_log_buffer_[address] + prob_hit_log_));
}

void VoxelMap::inflateObstacles()
{
    if (!initialized_)
    {
        return;
    }

    std::fill(inflated_buffer_.begin(), inflated_buffer_.end(), FREE);

    std::vector<Eigen::Vector3i> occupied_indices;
    occupied_indices.reserve(occupancy_log_buffer_.size() / 20);
    for (int address = 0; address < static_cast<int>(occupancy_log_buffer_.size()); ++address)
    {
        if (isLogOccupied(address))
        {
            occupied_indices.push_back(addressToIndex(address));
        }
    }

    // 膨胀步数为 $ceil(inflation_radius / resolution)$，例如 $0.5 / 0.2$ 会得到 3 层邻域。
    const int inflate_step = static_cast<int>(std::ceil(inflation_radius_ * inv_resolution_));
    for (const Eigen::Vector3i& occ_idx : occupied_indices)
    {
        for (int dx = -inflate_step; dx <= inflate_step; ++dx)
        {
            for (int dy = -inflate_step; dy <= inflate_step; ++dy)
            {
                for (int dz = -inflate_step; dz <= inflate_step; ++dz)
                {
                    const double distance = std::sqrt(static_cast<double>(dx * dx + dy * dy + dz * dz)) * resolution_;
                    if (distance > inflation_radius_ + 1e-6)
                    {
                        continue;
                    }

                    const Eigen::Vector3i neighbor = occ_idx + Eigen::Vector3i(dx, dy, dz);
                    if (!isInMap(neighbor))
                    {
                        continue;
                    }

                    const int address = toAddress(neighbor);
                    inflated_buffer_[address] = INFLATED;
                }
            }
        }
    }
}

bool VoxelMap::isOccupied(const Eigen::Vector3d& pos) const
{
    Eigen::Vector3i idx;
    if (!posToIndex(pos, idx))
    {
        return false;
    }
    return isLogOccupied(toAddress(idx));
}

bool VoxelMap::isInflatedOccupied(const Eigen::Vector3d& pos) const
{
    Eigen::Vector3i idx;
    if (!posToIndex(pos, idx))
    {
        return false;
    }

    const int address = toAddress(idx);
    return isLogOccupied(address) || inflated_buffer_[address] == INFLATED;
}

bool VoxelMap::isLineFree(const Eigen::Vector3d& p0,
                          const Eigen::Vector3d& p1,
                          double step_size) const
{
    if (!initialized_)
    {
        return false;
    }

    // 采样步长默认取半个体素，降低线段穿过窄障碍但采样点跳过的概率。
    const double step = step_size > 1e-6 ? step_size : resolution_ * 0.5;
    const double length = (p1 - p0).norm();
    const int sample_num = std::max(1, static_cast<int>(std::ceil(length / step)));

    for (int i = 0; i <= sample_num; ++i)
    {
        const double alpha = static_cast<double>(i) / static_cast<double>(sample_num);
        const Eigen::Vector3d p = p0 + alpha * (p1 - p0);

        // 线段离开局部地图时也认为不可直接通行，避免 planner 把未知区域当作安全区域。
        if (!isInMap(p) || isInflatedOccupied(p))
        {
            return false;
        }
    }

    return true;
}

std::vector<Eigen::Vector3d> VoxelMap::getOccupiedVoxelCenters() const
{
    std::vector<Eigen::Vector3d> centers;
    for (int address = 0; address < static_cast<int>(occupancy_log_buffer_.size()); ++address)
    {
        if (isLogOccupied(address))
        {
            centers.push_back(indexToPos(addressToIndex(address)));
        }
    }
    return centers;
}

std::vector<Eigen::Vector3d> VoxelMap::getInflatedVoxelCenters(bool include_occupied) const
{
    std::vector<Eigen::Vector3d> centers;
    for (int address = 0; address < static_cast<int>(inflated_buffer_.size()); ++address)
    {
        if (inflated_buffer_[address] == INFLATED || (include_occupied && isLogOccupied(address)))
        {
            centers.push_back(indexToPos(addressToIndex(address)));
        }
    }
    return centers;
}

bool VoxelMap::query(const Eigen::Vector3d& pos) const
{
    Eigen::Vector3i idx;
    if (!posToIndex(pos, idx))
    {
        // 对 GCOPTER/FIRI 走廊生成来说，局部地图外部按 blocked 处理，避免轨迹越过当前可感知范围。
        return true;
    }

    return isBlockedAddress(toAddress(idx), true);
}

void VoxelMap::getSurf(std::vector<Eigen::Vector3d>& points,
                       bool include_occupied) const
{
    if (!initialized_)
    {
        return;
    }

    // GCOPTER 原始地图只把最后一层 dilated wavefront 作为 surf。
    // FastNav 使用欧氏半径膨胀，因此这里提取 blocked 集合的边界：若体素 $v$ 的 6 邻域中存在 free / out-of-map，则 $v$ 是表面体素。
    points.reserve(points.size() + inflated_buffer_.size() / 10);
    for (int address = 0; address < static_cast<int>(inflated_buffer_.size()); ++address)
    {
        if (!isBlockedAddress(address, include_occupied))
        {
            continue;
        }

        const Eigen::Vector3i idx = addressToIndex(address);
        if (isSurfaceIndex(idx, include_occupied))
        {
            points.push_back(indexToPos(idx));
        }
    }
}

void VoxelMap::getSurfInBox(const Eigen::Vector3d& box_min,
                            const Eigen::Vector3d& box_max,
                            std::vector<Eigen::Vector3d>& points,
                            bool include_occupied) const
{
    if (!initialized_)
    {
        return;
    }

    Eigen::Vector3d min_corner = box_min.cwiseMax(origin_);
    Eigen::Vector3d max_corner = box_max.cwiseMin(getCorner());
    if ((max_corner.array() <= min_corner.array()).any())
    {
        return;
    }

    auto posToClampedIndex = [this](const Eigen::Vector3d& pos) {
        Eigen::Vector3i idx;
        const Eigen::Vector3d relative = (pos - origin_) * inv_resolution_;
        idx.x() = std::min(dim_.x() - 1, std::max(0, static_cast<int>(std::floor(relative.x()))));
        idx.y() = std::min(dim_.y() - 1, std::max(0, static_cast<int>(std::floor(relative.y()))));
        idx.z() = std::min(dim_.z() - 1, std::max(0, static_cast<int>(std::floor(relative.z()))));
        return idx;
    };

    const Eigen::Vector3i min_idx = posToClampedIndex(min_corner);
    const Eigen::Vector3i max_idx = posToClampedIndex(max_corner - Eigen::Vector3d::Constant(1.0e-9));

    const int box_voxel_num = std::max(1,
                                       (max_idx.x() - min_idx.x() + 1) *
                                           (max_idx.y() - min_idx.y() + 1) *
                                           (max_idx.z() - min_idx.z() + 1));
    points.reserve(points.size() + box_voxel_num / 10);

    for (int ix = min_idx.x(); ix <= max_idx.x(); ++ix)
    {
        for (int iy = min_idx.y(); iy <= max_idx.y(); ++iy)
        {
            for (int iz = min_idx.z(); iz <= max_idx.z(); ++iz)
            {
                const Eigen::Vector3i idx(ix, iy, iz);
                const int address = toAddress(idx);
                if (!isBlockedAddress(address, include_occupied))
                {
                    continue;
                }
                if (isSurfaceIndex(idx, include_occupied))
                {
                    points.push_back(indexToPos(idx));
                }
            }
        }
    }
}

int VoxelMap::toAddress(const Eigen::Vector3i& idx) const
{
    // 三维索引按 $address = ix * dim_y * dim_z + iy * dim_z + iz$ 压成一维数组下标。
    return idx.x() * dim_.y() * dim_.z() + idx.y() * dim_.z() + idx.z();
}

Eigen::Vector3i VoxelMap::addressToIndex(int address) const
{
    Eigen::Vector3i idx;
    idx.x() = address / (dim_.y() * dim_.z());
    const int rem = address % (dim_.y() * dim_.z());
    idx.y() = rem / dim_.z();
    idx.z() = rem % dim_.z();
    return idx;
}

bool VoxelMap::isLogOccupied(int address) const
{
    return address >= 0 &&
           address < static_cast<int>(occupancy_log_buffer_.size()) &&
           occupancy_log_buffer_[address] > min_occupancy_log_;
}

bool VoxelMap::isBlockedAddress(int address, bool include_occupied) const
{
    if (address < 0 || address >= static_cast<int>(inflated_buffer_.size()))
    {
        return true;
    }

    return inflated_buffer_[address] == INFLATED ||
           (include_occupied && isLogOccupied(address));
}

bool VoxelMap::isSurfaceIndex(const Eigen::Vector3i& idx, bool include_occupied) const
{
    static const Eigen::Vector3i kNeighbors[6] = {
        Eigen::Vector3i(1, 0, 0),
        Eigen::Vector3i(-1, 0, 0),
        Eigen::Vector3i(0, 1, 0),
        Eigen::Vector3i(0, -1, 0),
        Eigen::Vector3i(0, 0, 1),
        Eigen::Vector3i(0, 0, -1),
    };

    for (const Eigen::Vector3i& delta : kNeighbors)
    {
        const Eigen::Vector3i neighbor = idx + delta;
        if (!isInMap(neighbor))
        {
            return true;
        }

        if (!isBlockedAddress(toAddress(neighbor), include_occupied))
        {
            return true;
        }
    }

    return false;
}

void VoxelMap::shiftBuffersForNewOrigin(const Eigen::Vector3d& new_origin)
{
    if (!origin_initialized_)
    {
        origin_ = new_origin;
        origin_initialized_ = true;
        return;
    }

    Eigen::Vector3i shift;
    shift.x() = static_cast<int>(std::llround((new_origin.x() - origin_.x()) * inv_resolution_));
    shift.y() = static_cast<int>(std::llround((new_origin.y() - origin_.y()) * inv_resolution_));
    shift.z() = static_cast<int>(std::llround((new_origin.z() - origin_.z()) * inv_resolution_));

    if (shift == Eigen::Vector3i::Zero())
    {
        origin_ = new_origin;
        return;
    }

    if (std::abs(shift.x()) >= dim_.x() ||
        std::abs(shift.y()) >= dim_.y() ||
        std::abs(shift.z()) >= dim_.z())
    {
        origin_ = new_origin;
        reset();
        return;
    }

    std::vector<double> shifted_log(occupancy_log_buffer_.size(), unknown_log_);
    std::vector<int8_t> shifted_inflated(inflated_buffer_.size(), FREE);

    for (int x = 0; x < dim_.x(); ++x)
    {
        for (int y = 0; y < dim_.y(); ++y)
        {
            for (int z = 0; z < dim_.z(); ++z)
            {
                const Eigen::Vector3i old_idx(x, y, z);
                const Eigen::Vector3i new_idx = old_idx - shift;
                if (!isInMap(new_idx))
                {
                    continue;
                }

                shifted_log[toAddress(new_idx)] = occupancy_log_buffer_[toAddress(old_idx)];
                shifted_inflated[toAddress(new_idx)] = inflated_buffer_[toAddress(old_idx)];
            }
        }
    }

    origin_ = new_origin;
    occupancy_log_buffer_.swap(shifted_log);
    inflated_buffer_.swap(shifted_inflated);
}

}  // namespace fastnav_mapping
