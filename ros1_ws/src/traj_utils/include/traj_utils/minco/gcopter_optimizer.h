#pragma once

// FastNav 对 GCOPTER/MINCO 优化器的统一入口。
// 这里不重新实现算法，只把 vendor 到 traj_utils/minco/gcopter 的 header-only 优化器暴露给 planner。
// 核心类型包括 gcopter::GCOPTER_PolytopeSFC、minco::MINCO_S3NU 和 Trajectory<5>。
#include "traj_utils/minco/gcopter/geo_utils.hpp"
#include "traj_utils/minco/gcopter/gcopter.hpp"
