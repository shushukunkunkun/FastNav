#include "traj_utils/corridor/safe_corridor_generator.h"

#include <algorithm>
#include <cmath>
#include <deque>

#include <traj_utils/minco/gcopter/firi.hpp>

namespace traj_utils
{

namespace
{

bool polytopesOverlap(const Eigen::MatrixX4d& hpoly0,
                      const Eigen::MatrixX4d& hpoly1,
                      double eps)
{
    // 检查两个 H-polytope 是否相交：联合约束存在内点时，$\mathcal{P}_0\cap\mathcal{P}_1\ne\emptyset$。
    const int m = hpoly0.rows();
    const int n = hpoly1.rows();
    Eigen::MatrixX4d A(m + n, 4);
    Eigen::VectorXd b(m + n);
    Eigen::Vector4d c;
    Eigen::Vector4d x;

    A.leftCols<3>().topRows(m) = hpoly0.leftCols<3>();
    A.leftCols<3>().bottomRows(n) = hpoly1.leftCols<3>();
    A.rightCols<1>().setConstant(1.0);
    b.topRows(m) = -hpoly0.rightCols<1>();
    b.bottomRows(n) = -hpoly1.rightCols<1>();
    c.setZero();
    c(3) = -1.0;

    const double minmaxsd = sdlp::linprog<4>(c, A, b, x);
    return minmaxsd < -eps && !std::isinf(minmaxsd);
}

}  // namespace

void SafeCorridorGenerator::setConfig(const Config& config)
{
    config_ = config;
    config_.progress = std::max(1.0e-3, config_.progress);
    config_.range = std::max(1.0e-3, config_.range);
    config_.eps = std::max(1.0e-9, config_.eps);
}

bool SafeCorridorGenerator::generate(const std::vector<Eigen::Vector3d>& path,
                                     const std::vector<Eigen::Vector3d>& obstacle_surface_points,
                                     const Eigen::Vector3d& low_corner,
                                     const Eigen::Vector3d& high_corner,
                                     std::vector<Eigen::MatrixX4d>& hpolys)
{
    last_error_.clear();
    hpolys.clear();

    if (path.size() < 2)
    {
        last_error_ = "Path needs at least two points for corridor generation.";
        return false;
    }

    if ((high_corner.array() <= low_corner.array()).any())
    {
        last_error_ = "Invalid map bounds for corridor generation.";
        return false;
    }

    if (!convexCover(path, obstacle_surface_points, low_corner, high_corner, hpolys))
    {
        return false;
    }

    if (config_.enable_shortcut)
    {
        shortCut(hpolys);
    }

    return !hpolys.empty();
}

std::string SafeCorridorGenerator::lastError() const
{
    return last_error_;
}

bool SafeCorridorGenerator::convexCover(const std::vector<Eigen::Vector3d>& path,
                                        const std::vector<Eigen::Vector3d>& obstacle_surface_points,
                                        const Eigen::Vector3d& low_corner,
                                        const Eigen::Vector3d& high_corner,
                                        std::vector<Eigen::MatrixX4d>& hpolys)
{
    // bd 表示局部盒约束，形式为 $h_0x+h_1y+h_2z+h_3\le0$。
    Eigen::Matrix<double, 6, 4> bd = Eigen::Matrix<double, 6, 4>::Zero();
    bd(0, 0) = 1.0;
    bd(1, 0) = -1.0;
    bd(2, 1) = 1.0;
    bd(3, 1) = -1.0;
    bd(4, 2) = 1.0;
    bd(5, 2) = -1.0;

    Eigen::MatrixX4d hp;
    Eigen::MatrixX4d gap;
    Eigen::Vector3d a;
    Eigen::Vector3d b = path.front();
    std::vector<Eigen::Vector3d> valid_points;
    valid_points.reserve(obstacle_surface_points.size());

    const int path_size = static_cast<int>(path.size());
    for (int i = 1; i < path_size;)
    {
        a = b;
        if ((a - path[i]).norm() > config_.progress)
        {
            b = a + (path[i] - a).normalized() * config_.progress;
        }
        else
        {
            b = path[i];
            ++i;
        }

        // 以当前路径段 $[a,b]$ 为中心裁剪局部障碍点，避免把整张地图点云都交给 FIRI。
        bd(0, 3) = -std::min(std::max(a(0), b(0)) + config_.range, high_corner(0));
        bd(1, 3) = +std::max(std::min(a(0), b(0)) - config_.range, low_corner(0));
        bd(2, 3) = -std::min(std::max(a(1), b(1)) + config_.range, high_corner(1));
        bd(3, 3) = +std::max(std::min(a(1), b(1)) - config_.range, low_corner(1));
        bd(4, 3) = -std::min(std::max(a(2), b(2)) + config_.range, high_corner(2));
        bd(5, 3) = +std::max(std::min(a(2), b(2)) - config_.range, low_corner(2));

        valid_points.clear();
        for (const Eigen::Vector3d& point : obstacle_surface_points)
        {
            if ((bd.leftCols<3>() * point + bd.rightCols<1>()).maxCoeff() < 0.0)
            {
                valid_points.push_back(point);
            }
        }

        Eigen::Matrix3Xd pc(3, static_cast<int>(valid_points.size()));
        for (int j = 0; j < static_cast<int>(valid_points.size()); ++j)
        {
            pc.col(j) = valid_points[j];
        }

        // FIRI 生成包含线段 $[a,b]$ 且避开障碍点 pc 的凸多面体。
        if (!firi::firi(bd, pc, a, b, hp))
        {
            last_error_ = "FIRI failed to generate a safe corridor segment.";
            return false;
        }

        if (!hpolys.empty())
        {
            const Eigen::Vector4d ah(a(0), a(1), a(2), 1.0);
            const int active_constraints =
                ((hp * ah).array() > -config_.eps).cast<int>().sum() +
                ((hpolys.back() * ah).array() > -config_.eps).cast<int>().sum();

            // 若相邻走廊在连接点 $a$ 附近贴得太紧，则补一个 gap polytope 保证走廊序列连通。
            if (active_constraints >= 3)
            {
                if (firi::firi(bd, pc, a, a, gap, 1))
                {
                    hpolys.push_back(gap);
                }
            }
        }

        hpolys.push_back(hp);
    }

    return true;
}

void SafeCorridorGenerator::shortCut(std::vector<Eigen::MatrixX4d>& hpolys)
{
    if (hpolys.empty())
    {
        return;
    }

    std::vector<Eigen::MatrixX4d> temp = hpolys;
    if (temp.size() == 1)
    {
        temp.insert(temp.begin(), temp.front());
    }

    hpolys.clear();
    std::deque<int> ids;
    ids.push_front(static_cast<int>(temp.size()) - 1);

    for (int i = static_cast<int>(temp.size()) - 1; i >= 0; --i)
    {
        for (int j = 0; j < i; ++j)
        {
            const bool overlap = (j >= i - 1) || polytopesOverlap(temp[i], temp[j], 0.01);
            if (overlap)
            {
                ids.push_front(j);
                i = j + 1;
                break;
            }
        }
    }

    for (const int id : ids)
    {
        hpolys.push_back(temp[id]);
    }
}

}  // namespace traj_utils
