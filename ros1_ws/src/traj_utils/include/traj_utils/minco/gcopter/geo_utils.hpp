/*
 * FastNav 注释：geo_utils.hpp 收集凸几何工具函数。
 * GCOPTER 的安全走廊依赖两类表示：顶点表示 $V$ 和半空间表示 $Hx\le h$，几何工具负责二者转换、交集和距离计算。
 * 这些工具通常服务于 FIRI、QuickHull、SDLP 和多面体可视化。
 * 对 FastNav 来说，接入 MINCO 初期不需要直接使用；接入凸安全走廊时再重点看这里。
 */

#pragma once
#include "quickhull.hpp"
#include "sdlp.hpp"

#include <Eigen/Eigen>

#include <cfloat>
#include <cstdint>
#include <set>
#include <chrono>

namespace geo_utils
{

    // Each row of hPoly is defined by h0, h1, h2, h3 as
    // h0*x + h1*y + h2*z + h3 <= 0
    inline bool findInterior(const Eigen::MatrixX4d &hPoly,
                             Eigen::Vector3d &interior)
    {
        // 寻找多面体内部点。给定半空间 $h_i^T[x,y,z,1]^T\le0$，这里通过低维 LP 最大化内点到所有平面的最小归一化距离。
        const int m = hPoly.rows();

        Eigen::MatrixX4d A(m, 4);
        Eigen::VectorXd b(m);
        Eigen::Vector4d c, x;
        const Eigen::ArrayXd hNorm = hPoly.leftCols<3>().rowwise().norm();
        A.leftCols<3>() = hPoly.leftCols<3>().array().colwise() / hNorm;
        A.rightCols<1>().setConstant(1.0);
        b = -hPoly.rightCols<1>().array() / hNorm;
        c.setZero();
        c(3) = -1.0;

        const double minmaxsd = sdlp::linprog<4>(c, A, b, x);
        // x 的前三维是候选内点，第四维可理解为安全裕度 $d$；若 LP 返回有效正裕度，则多面体有非空内部。
        interior = x.head<3>();

        return minmaxsd < 0.0 && !std::isinf(minmaxsd);
    }

    inline bool overlap(const Eigen::MatrixX4d &hPoly0,
                        const Eigen::MatrixX4d &hPoly1,
                        const double eps = 1.0e-6)

    {
        // 判断两个 H-representation 多面体是否有交集：检查联合约束 $H_0x\le h_0, H_1x\le h_1$ 是否存在内点。
        // shortCut() 用这个结果决定是否可以跳过中间走廊段。
        const int m = hPoly0.rows();
        const int n = hPoly1.rows();
        Eigen::MatrixX4d A(m + n, 4);
        Eigen::Vector4d c, x;
        Eigen::VectorXd b(m + n);
        A.leftCols<3>().topRows(m) = hPoly0.leftCols<3>();
        A.leftCols<3>().bottomRows(n) = hPoly1.leftCols<3>();
        A.rightCols<1>().setConstant(1.0);
        b.topRows(m) = -hPoly0.rightCols<1>();
        b.bottomRows(n) = -hPoly1.rightCols<1>();
        c.setZero();
        c(3) = -1.0;

        const double minmaxsd = sdlp::linprog<4>(c, A, b, x);

        // minmaxsd < -eps 表示联合多面体存在至少 eps 深度的内点，即 $\mathcal{P}_0\cap\mathcal{P}_1\ne\emptyset$。
        return minmaxsd < -eps && !std::isinf(minmaxsd);
    }

    struct filterLess
    {
        inline bool operator()(const Eigen::Vector3d &l,
                               const Eigen::Vector3d &r) const
        {
            return l(0) < r(0) ||
                   (l(0) == r(0) &&
                    (l(1) < r(1) ||
                     (l(1) == r(1) &&
                      l(2) < r(2))));
        }
    };

    inline void filterVs(const Eigen::Matrix3Xd &rV,
                         const double &epsilon,
                         Eigen::Matrix3Xd &fV)
    {
        // 顶点枚举会产生数值重复点，这里按分辨率 $res$ 对顶点量化后去重。
        const double mag = std::max(fabs(rV.maxCoeff()), fabs(rV.minCoeff()));
        const double res = mag * std::max(fabs(epsilon) / mag, DBL_EPSILON);
        std::set<Eigen::Vector3d, filterLess> filter;
        fV = rV;
        int offset = 0;
        Eigen::Vector3d quanti;
        for (int i = 0; i < rV.cols(); i++)
        {
            quanti = (rV.col(i) / res).array().round();
            if (filter.find(quanti) == filter.end())
            {
                filter.insert(quanti);
                fV.col(offset) = rV.col(i);
                offset++;
            }
        }
        fV = fV.leftCols(offset).eval();
        return;
    }

    // Each row of hPoly is defined by h0, h1, h2, h3 as
    // h0*x + h1*y + h2*z + h3 <= 0
    // proposed epsilon is 1.0e-6
    inline void enumerateVs(const Eigen::MatrixX4d &hPoly,
                            const Eigen::Vector3d &inner,
                            Eigen::Matrix3Xd &vPoly,
                            const double epsilon = 1.0e-6)
    {
        // 把半空间表示 $Hx+h\le0$ 转成顶点表示。先以内部点 inner 为中心做极对偶变换，再用 QuickHull 求凸包。
        // 这一步主要用于可视化和多面体几何运算，不改变走廊本身。
        const Eigen::VectorXd b = -hPoly.rightCols<1>() - hPoly.leftCols<3>() * inner;
        // b 是 inner 到各半空间边界的有符号裕度；只有 $b_i>0$ 时 inner 才确实在多面体内部。
        const Eigen::Matrix<double, 3, -1, Eigen::ColMajor> A =
            (hPoly.leftCols<3>().array().colwise() / b.array()).transpose();

        quickhull::QuickHull<double> qh;
        const double qhullEps = std::min(epsilon, quickhull::defaultEps<double>());
        // CCW is false because the normal in quickhull towards interior
        const auto cvxHull = qh.getConvexHull(A.data(), A.cols(), false, true, qhullEps);
        const auto &idBuffer = cvxHull.getIndexBuffer();
        const int hNum = idBuffer.size() / 3;
        Eigen::Matrix3Xd rV(3, hNum);
        Eigen::Vector3d normal, point, edge0, edge1;
        for (int i = 0; i < hNum; i++)
        {
            point = A.col(idBuffer[3 * i + 1]);
            edge0 = point - A.col(idBuffer[3 * i]);
            edge1 = A.col(idBuffer[3 * i + 2]) - point;
            normal = edge0.cross(edge1); //cross in CW gives an outter normal
            rV.col(i) = normal / normal.dot(point);
        }
        filterVs(rV, epsilon, vPoly);
        vPoly = (vPoly.array().colwise() + inner.array()).eval();
        return;
    }

    // Each row of hPoly is defined by h0, h1, h2, h3 as
    // h0*x + h1*y + h2*z + h3 <= 0
    // proposed epsilon is 1.0e-6
    inline bool enumerateVs(const Eigen::MatrixX4d &hPoly,
                            Eigen::Matrix3Xd &vPoly,
                            const double epsilon = 1.0e-6)
    {
        Eigen::Vector3d inner;
        if (findInterior(hPoly, inner))
        {
            enumerateVs(hPoly, inner, vPoly, epsilon);
            return true;
        }
        else
        {
            return false;
        }
    }

} // namespace geo_utils
