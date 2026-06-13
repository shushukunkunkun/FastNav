#include "fastnav_planner/optimizer/path_optimizer.h"

#include <algorithm>
#include <cmath>

#include <ros/time.h>

namespace fastnav_planner
{

void PathOptimizer::setConfig(const Config& config)
{
    config_ = config;
    config_.line_check_step = std::max(1.0e-3, config_.line_check_step);
    config_.minco_sample_dt = std::max(1.0e-3, config_.minco_sample_dt);
    config_.max_retry = std::max(1, config_.max_retry);
    config_.corridor_range_retry_scale = std::max(0.1, config_.corridor_range_retry_scale);
    config_.corridor_progress_retry_scale = std::max(0.1, config_.corridor_progress_retry_scale);
    config_.weight_time_retry_scale = std::max(0.1, config_.weight_time_retry_scale);
    config_.penalty_pos_retry_scale = std::max(0.1, config_.penalty_pos_retry_scale);
    config_.penalty_vel_retry_scale = std::max(0.1, config_.penalty_vel_retry_scale);
    config_.penalty_body_rate_retry_scale = std::max(0.1, config_.penalty_body_rate_retry_scale);
    config_.penalty_tilt_retry_scale = std::max(0.1, config_.penalty_tilt_retry_scale);
    config_.penalty_thrust_retry_scale = std::max(0.1, config_.penalty_thrust_retry_scale);
    corridor_generator_.setConfig(config_.corridor);
    minco_optimizer_.setConfig(config_.minco);
    feasibility_checker_.setConfig(config_.feasibility);
}

// 完整后端优化入口：将前端 A* / guide path 输出的几何路径优化成可执行 MINCO 轨迹。
// 输入 raw_path 是一串离散航点 $\{p_i\}_{i=0}^{N}$，它只保证在 voxel map 上连通；
// 本函数依次完成：
// 1. shortcut：删除可直连的冗余点，得到走廊锚点 $\{q_i\}$；
// 2. surface extraction：从路径附近提取障碍表面点 $\mathcal{P}_{obs}$；
// 3. FIRI safe corridor：围绕 $\{q_i\}$ 生成凸多面体走廊 $H_k[x^T,1]^T \le 0$；
// 4. MINCO/GCOPTER：在走廊约束和动力学代价下求连续轨迹 $p(t)$；
// 5. fine check：离散采样检查碰撞和动力学约束，失败则按 violation 类型重试。
//
// result 是本函数的完整输出容器：
// - result.raw_path 保存输入 A* 路径；
// - result.shortcut_path 保存 corridor 锚点；
// - result.corridors 保存每段凸走廊半空间；
// - result.minco_traj 保存连续轨迹；
// - result.sampled_path 保存 $p(t)$ 的采样点，仅用于 RViz / Path topic 调试。
// 因此 planner 主流程应优先调用这个函数，而不是下面的简化版 optimize()。
bool PathOptimizer::optimizeTrajectory(const std::vector<Eigen::Vector3d>& raw_path,
                                       const std::shared_ptr<fastnav_mapping::VoxelMap>& map,
                                       const fastnav::MincoGcopterOptimizer::BoundaryState& start_state,
                                       const fastnav::MincoGcopterOptimizer::BoundaryState& goal_state,
                                       OptimizationResult& result,
                                       size_t preserve_prefix_size,
                                       bool touch_goal,
                                       const std::function<bool()>& preempt_requested)
{
    last_error_.clear();
    result.clear();
    result.raw_path = raw_path;

    if (raw_path.empty())
    {
        last_error_ = "Raw path is empty.";
        return false;
    }

    if (!config_.enable)
    {
        // 后端关闭时，不做 shortcut / corridor / MINCO，直接把 raw_path 当作几何输出。
        // 这主要用于调试前端 A*，不适合作为最终飞行轨迹。
        result.shortcut_path = raw_path;
        result.sampled_path = raw_path;
        return true;
    }

    if (!map)
    {
        last_error_ = "VoxelMap is not set.";
        return false;
    }

    if (preempt_requested && preempt_requested())
    {
        last_error_ = "Path optimization preempted before shortcut.";
        return false;
    }

    if (config_.shortcut)
    {
        // shortcut 的目标不是改变拓扑，而是把 A* 中同一条可直连走廊上的密集 voxel 点删掉。
        // 判断条件为 $lineFree(p_i,p_j)=true$，则可以用端点 $p_i,p_j$ 代表中间一串点。
        // preserve_prefix_size 用于保护旧轨迹前缀，避免 REPLAN 时把需要连续接轨的前缀删掉。
        const ros::WallTime shortcut_start = ros::WallTime::now();
        if (!shortcutPath(raw_path, map, result.shortcut_path, preserve_prefix_size))
        {
            result.shortcut_ms += (ros::WallTime::now() - shortcut_start).toSec() * 1000.0;
            return false;
        }
        result.shortcut_ms += (ros::WallTime::now() - shortcut_start).toSec() * 1000.0;
    }
    else
    {
        result.shortcut_path = raw_path;
    }
    result.sampled_path = result.shortcut_path;

    if (!config_.minco_enable)
    {
        // 只做几何 shortcut，不做连续轨迹优化。
        // 此时 result.sampled_path 仍是 shortcut_path，控制端不应把它当作连续 $p(t)$ 执行。
        return true;
    }

    if (result.shortcut_path.size() < 2)
    {
        last_error_ = "Shortcut path needs at least two points for MINCO.";
        return !config_.require_minco;
    }

    fastnav::MincoGcopterOptimizer::BoundaryState minco_start = start_state;
    fastnav::MincoGcopterOptimizer::BoundaryState minco_goal = goal_state;
    // MINCO 的端点位置必须与走廊路径首末点一致。
    // 速度、加速度边界仍沿用 FSM/manager 给出的 $v_0,a_0,v_T,a_T$，
    // 因此这里只覆盖位置，不覆盖 start_state / goal_state 中的动力学边界。
    minco_start.pos = result.shortcut_path.front();
    minco_goal.pos = result.shortcut_path.back();
    minco_optimizer_.setCancelCallback(preempt_requested);

    // 为了避免每次 retry 都重新扫描整张 voxel map，先按最大可能 corridor range
    // 在 shortcut path 附近提取一次障碍表面点。后续每轮 FIRI 只在这些局部点上构造分离平面。
    // 若 retry 会放大 range，则先计算 $R_{max}=\max_i R \cdot s_R^i$。
    double max_corridor_range = config_.corridor.range;
    for (int attempt = 1; attempt < config_.max_retry; ++attempt)
    {
        max_corridor_range = std::max(max_corridor_range,
                                      config_.corridor.range *
                                          std::pow(config_.corridor_range_retry_scale, attempt));
    }

    std::vector<Eigen::Vector3d> obstacle_surface_points;
    const ros::WallTime surface_start = ros::WallTime::now();
    // obstacle_surface_points 近似为 $\mathcal{P}_{obs}=\{p \mid dist(p,path)\le R_{max}\}$。
    // safe corridor 只需要路径附近障碍；远处障碍不会参与当前段的半空间约束。
    collectSurfacePointsForPath(result.shortcut_path,
                                map,
                                max_corridor_range,
                                obstacle_surface_points);
    result.corridor_ms += (ros::WallTime::now() - surface_start).toSec() * 1000.0;
    if (preempt_requested && preempt_requested())
    {
        last_error_ = "Path optimization preempted after surface extraction.";
        minco_optimizer_.setCancelCallback(std::function<bool()>());
        return false;
    }

    std::vector<Eigen::MatrixX4d> cached_corridors;
    bool has_cached_corridor = false;
    bool regenerate_corridor = true;

    // retry 循环与 EGO-v2 / GCOPTER 的思路一致：
    // - 第 1 次使用默认 corridor 和 MINCO 权重；
    // - 后续若 fine check 失败，则根据失败类型决定是否重新生成 corridor；
    // - 动力学失败通常只调大/调节 MINCO 代价，不必重跑 FIRI；
    // - 碰撞或空间类失败说明当前 corridor 支撑不足，需要重新生成或放大 corridor。
    // SEEME: 以下部分的代码构成了GCOPTER优化的核心链路
    for (int attempt = 0; attempt < config_.max_retry; ++attempt)
    {
        if (preempt_requested && preempt_requested())
        {
            last_error_ = "Path optimization preempted before retry.";
            minco_optimizer_.setCancelCallback(std::function<bool()>());
            return false;
        }

        const double corridor_range_scale = std::pow(config_.corridor_range_retry_scale, attempt); // NOTE: 单个轨迹的感知范围
        const double corridor_progress_scale = std::pow(config_.corridor_progress_retry_scale, attempt); // NOTE: 单段轨迹的推进长度
        const double weight_time_scale = std::pow(config_.weight_time_retry_scale, attempt);
        const double penalty_pos_scale = std::pow(config_.penalty_pos_retry_scale, attempt);
        const double penalty_vel_scale = std::pow(config_.penalty_vel_retry_scale, attempt);
        const double penalty_body_rate_scale = std::pow(config_.penalty_body_rate_retry_scale, attempt);
        const double penalty_tilt_scale = std::pow(config_.penalty_tilt_retry_scale, attempt);
        const double penalty_thrust_scale = std::pow(config_.penalty_thrust_retry_scale, attempt);

        // 每次 retry 可放大 corridor 的感知范围和单段推进长度：
        // $R_i = R_0 s_R^i$，$L_i = L_0 s_L^i$。
        // range 越大，FIRI 可用障碍表面点越多；progress 越大，单个 corridor 段覆盖路径越长。
        traj_utils::SafeCorridorGenerator::Config attempt_corridor_config = config_.corridor;
        attempt_corridor_config.range *= corridor_range_scale;
        attempt_corridor_config.progress *= corridor_progress_scale;
        corridor_generator_.setConfig(attempt_corridor_config);

        std::vector<Eigen::MatrixX4d> attempt_corridors;
        if (!has_cached_corridor || regenerate_corridor)
        {
            const ros::WallTime corridor_start = ros::WallTime::now();
            // generateCorridor() 以 shortcut_path 为中心线，为每段路径生成一个凸多面体。
            // 每个多面体用矩阵 $H \in \mathbb{R}^{m \times 4}$ 表示，
            // 行 $[n_x,n_y,n_z,d]$ 对应半空间 $n^T x + d \le 0$。
            if (!generateCorridor(result.shortcut_path, map, obstacle_surface_points, attempt_corridors))
            {
                result.corridor_ms += (ros::WallTime::now() - corridor_start).toSec() * 1000.0;
                last_error_ = "Attempt " + std::to_string(attempt + 1) +
                              " corridor failed: " + last_error_;
                regenerate_corridor = true;
                continue;
            }
            result.corridor_ms += (ros::WallTime::now() - corridor_start).toSec() * 1000.0;
            cached_corridors = attempt_corridors;
            has_cached_corridor = true;
            regenerate_corridor = false;
            // 即便后续 MINCO 或 fine check 失败，也保留本轮 corridor，
            // 便于 debug recorder 复盘“优化器被限制在哪个凸走廊内”。
            result.corridors = attempt_corridors;
        }
        else
        {
            // 如果上一轮只是速度/加速度等动力学 fine check 失败，则 corridor 本身仍可复用。
            // 这样避免昂贵的 FIRI 重复计算，只重新求解 MINCO 代价。
            attempt_corridors = cached_corridors;
            result.corridors = attempt_corridors;
        }

        if (preempt_requested && preempt_requested())
        {
            last_error_ = "Path optimization preempted after corridor generation.";
            minco_optimizer_.setCancelCallback(std::function<bool()>());
            return false;
        }

        fastnav::MincoGcopterOptimizer::Config attempt_minco_config = config_.minco;
        // MINCO retry 通过调整目标函数权重改变优化偏好。
        // 例如 penalty_pos / penalty_vel 变大时，优化器会更强地惩罚走廊外位置和速度超限；
        // weight_time 改变后，轨迹时间 $T$ 的代价也随之变化。
        attempt_minco_config.weight_time *= weight_time_scale;
        attempt_minco_config.penalty_pos *= penalty_pos_scale;
        attempt_minco_config.penalty_vel *= penalty_vel_scale;
        attempt_minco_config.penalty_body_rate *= penalty_body_rate_scale;
        attempt_minco_config.penalty_tilt *= penalty_tilt_scale;
        attempt_minco_config.penalty_thrust *= penalty_thrust_scale;
        minco_optimizer_.setConfig(attempt_minco_config);

        fastnav::MincoTraj attempt_traj;
        result.minco_retry_count = attempt + 1;
        const ros::WallTime minco_start_time = ros::WallTime::now();
        // MINCO/GCOPTER 优化
        // 目标是在满足走廊 $H_k[p(t)^T,1]^T \le 0$、端点 $p,v,a$ 约束、
        // 以及动力学惩罚的情况下，求一条平滑轨迹 $p(t)$。
        if (!minco_optimizer_.optimize(minco_start, minco_goal, attempt_corridors, attempt_traj))
        {
            result.minco_ms += (ros::WallTime::now() - minco_start_time).toSec() * 1000.0;
            last_error_ = "Attempt " + std::to_string(attempt + 1) +
                          " MINCO failed: " + minco_optimizer_.lastError();
            if (preempt_requested && preempt_requested())
            {
                minco_optimizer_.setCancelCallback(std::function<bool()>());
                return false;
            }
            continue;
        }
        result.minco_ms += (ros::WallTime::now() - minco_start_time).toSec() * 1000.0;
        // fine check 之前先缓存本轮 MINCO 结果。若后续碰撞/动力学检查失败，
        // debug recorder 仍然能保存这条“失败轨迹”用于离线查看 $p(t),v(t),a(t)$。
        result.corridors = attempt_corridors;
        result.minco_traj = attempt_traj;
        result.has_minco = attempt_traj.valid();
        result.sampled_path = attempt_traj.samplePositions(config_.minco_sample_dt);
        if (preempt_requested && preempt_requested())
        {
            last_error_ = "Path optimization preempted after MINCO.";
            minco_optimizer_.setCancelCallback(std::function<bool()>());
            return false;
        }

        TrajectoryFeasibilityChecker::Result feasibility;
        const ros::WallTime fine_check_start = ros::WallTime::now();
        // fine check 是优化后的最后一道硬检查：
        // 按 sample_dt 对 $p(t),v(t),a(t),j(t)$ 采样，
        // 碰撞检查使用 voxel map，动力学检查使用 $||v||\le v_{max}$、$||a||\le a_{max}$ 等上限。
        // 非最终 local target 可只检查前 $2/3$，最终目标则由 touch_goal 要求检查完整轨迹。
        if (!feasibility_checker_.check(attempt_traj, map, feasibility, touch_goal))
        {
            result.fine_check_ms += (ros::WallTime::now() - fine_check_start).toSec() * 1000.0;
            last_error_ = "Attempt " + std::to_string(attempt + 1) +
                          " fine check failed: " + feasibility.message;
            // collision / position 类失败说明当前空间走廊可能不合理，需要重建 corridor；
            // velocity / acceleration / jerk 类失败通常只是时间或权重问题，可复用 corridor 继续优化。
            regenerate_corridor = shouldRegenerateCorridorAfterViolation(feasibility.violation_type);
            continue;
        }
        result.fine_check_ms += (ros::WallTime::now() - fine_check_start).toSec() * 1000.0;

        result.corridors = attempt_corridors;
        result.minco_traj = attempt_traj;
        result.has_minco = true;
        // sampled_path 只用于 RViz / nav_msgs::Path 调试显示；真正控制执行使用 minco_traj 中的连续 $p(t)$。
        result.sampled_path = result.minco_traj.samplePositions(config_.minco_sample_dt);
        if (result.sampled_path.empty())
        {
            result.has_minco = false;
            last_error_ = "Attempt " + std::to_string(attempt + 1) +
                          " MINCO trajectory sampling returned empty path.";
            continue;
        }

        last_error_.clear();
        minco_optimizer_.setCancelCallback(std::function<bool()>());
        return true;
    }

    if (!config_.require_minco)
    {
        // 若允许 MINCO 失败后降级，则返回 shortcut 几何路径。
        // FastNav 当前用于真实执行时通常 require_minco=true，因为控制链路需要连续轨迹。
        result.sampled_path = result.shortcut_path;
        minco_optimizer_.setCancelCallback(std::function<bool()>());
        return true;
    }

    minco_optimizer_.setCancelCallback(std::function<bool()>());
    return false;
}

// 简化几何优化入口，保留给旧接口、单元测试或快速调试使用。
// 它只接收 raw_path 和 map，不接收速度/加速度边界，因此默认边界为零：
// $v_0=a_0=v_T=a_T=0$。这会让 MINCO 端点动力学约束过于简单，
// 不适合执行中重规划，因为执行中重规划应使用 FSM 给出的
// $p(t_s),v(t_s),a(t_s)$ 作为起点边界。
//
// 内部仍然调用完整 optimizeTrajectory()，但只把 result.sampled_path 或
// result.shortcut_path 拷贝到 optimized_path，丢弃 corridor、MINCO 时间戳、
// retry timing 等详细信息。因此主 planner 流程不应使用这个函数。
bool PathOptimizer::optimize(const std::vector<Eigen::Vector3d>& raw_path,
                             const std::shared_ptr<fastnav_mapping::VoxelMap>& map,
                             std::vector<Eigen::Vector3d>& optimized_path)
{
    optimized_path.clear();

    // 这里没有 odom / traj 边界信息，只能用 raw_path 首末点填位置边界。
    // vel / acc 保持默认零值，相当于要求轨迹首末端静止。
    fastnav::MincoGcopterOptimizer::BoundaryState start;
    fastnav::MincoGcopterOptimizer::BoundaryState goal;
    if (!raw_path.empty())
    {
        start.pos = raw_path.front();
        goal.pos = raw_path.back();
    }

    OptimizationResult result;
    const bool success = optimizeTrajectory(raw_path, map, start, goal, result);
    if (!success)
    {
        return false;
    }

    optimized_path = result.sampled_path.empty() ? result.shortcut_path : result.sampled_path;
    return !optimized_path.empty();
}

std::string PathOptimizer::lastError() const
{
    return last_error_;
}

bool PathOptimizer::shortcutPath(const std::vector<Eigen::Vector3d>& raw_path,
                                 const std::shared_ptr<fastnav_mapping::VoxelMap>& map,
                                 std::vector<Eigen::Vector3d>& optimized_path,
                                 size_t preserve_prefix_size) const
{
    if (raw_path.size() <= 2)
    {
        optimized_path = raw_path;
        return true;
    }

    // 从当前点 $p_i$ 开始，寻找最远的 $p_j$，使线段 $p_i -> p_j$ 不穿过膨胀障碍。
    // 这样可以保留路径拓扑，同时删除 A* 栅格搜索带来的锯齿点。
    // 当 preserve_prefix_size > 1 时，前缀点来自当前旧 MINCO 轨迹的剩余安全段，必须原样保留；
    // shortcut 只从前缀末端开始处理后续 A* 桥接段，避免破坏新旧轨迹切换的连续性。
    const size_t preserve_num = std::min(preserve_prefix_size, raw_path.size());
    size_t i = 0;
    if (preserve_num > 1)
    {
        optimized_path.insert(optimized_path.end(), raw_path.begin(), raw_path.begin() + preserve_num);
        i = preserve_num - 1;
    }
    else
    {
        optimized_path.push_back(raw_path.front());
    }

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

// 为当前 shortcut path 提取附近障碍物表面点，供 FIRI / safe corridor 使用。
// FIRI 生成分离平面时只关心路径附近的障碍点集合 $\mathcal{P}_{obs}$，
// 不需要把整张局部地图的 surface voxel 都传进去，否则 corridor 生成会很慢。
// 这里先构造包含整条路径的轴对齐包围盒 $B=[p_{min}-r, p_{max}+r]$，
// 再调用 VoxelMap::getSurfInBox() 只提取 $B$ 内的 surface points。
void PathOptimizer::collectSurfacePointsForPath(
    const std::vector<Eigen::Vector3d>& path,
    const std::shared_ptr<fastnav_mapping::VoxelMap>& map,
    double range,
    std::vector<Eigen::Vector3d>& obstacle_surface_points) const
{
    obstacle_surface_points.clear();
    if (!map || path.empty())
    {
        return;
    }

    // 先计算 shortcut path 的 AABB：
    // $p_{min}=\min_i p_i$，$p_{max}=\max_i p_i$。
    // 这个盒子只覆盖路径本身，还没有包含周围障碍搜索半径。
    Eigen::Vector3d box_min = path.front();
    Eigen::Vector3d box_max = path.front();
    for (const Eigen::Vector3d& point : path)
    {
        box_min = box_min.cwiseMin(point);
        box_max = box_max.cwiseMax(point);
    }

    // 将路径 AABB 向外扩展 range，得到候选障碍盒：
    // $B=[p_{min}-r\mathbf{1}, p_{max}+r\mathbf{1}]$。
    // range 使用本轮 optimizeTrajectory() 中所有 retry 的最大 corridor range，
    // 因此 surface extraction 只做一次，后续 retry 直接复用 obstacle_surface_points。
    const double bounded_range = std::max(0.0, range);
    box_min.array() -= bounded_range;
    box_max.array() += bounded_range;

    // FIRI 每段还会在 SafeCorridorGenerator 内部按当前 range 做二次筛选；
    // 这里先用整条路径的最大 retry range 提取一次候选表面点，避免每轮 retry 扫描整张局部地图。
    // include_occupied=true 表示 occupied 和 inflated 的表面体素都作为障碍边界候选。
    map->getSurfInBox(box_min, box_max, obstacle_surface_points, true);
}

bool PathOptimizer::generateCorridor(const std::vector<Eigen::Vector3d>& path,
                                     const std::shared_ptr<fastnav_mapping::VoxelMap>& map,
                                     const std::vector<Eigen::Vector3d>& obstacle_surface_points,
                                     std::vector<Eigen::MatrixX4d>& corridors)
{
    corridors.clear();
    if (!map)
    {
        last_error_ = "VoxelMap is not set for corridor generation.";
        return false;
    }

    if (!corridor_generator_.generate(path,
                                      obstacle_surface_points,
                                      map->getOrigin(),
                                      map->getCorner(),
                                      corridors))
    {
        last_error_ = "Corridor generation failed: " + corridor_generator_.lastError();
        return false;
    }

    if (corridors.empty())
    {
        last_error_ = "Corridor generation returned empty corridor.";
        return false;
    }

    return true;
}

bool PathOptimizer::shouldRegenerateCorridorAfterViolation(const std::string& violation_type) const
{
    // 空间类失败说明当前 corridor / 几何约束附近不够安全，下一轮应扩大或重新生成走廊；
    // 动力学类失败如 velocity / acceleration / jerk 通常只需要调整 MINCO 时间或惩罚，不必重复 FIRI。
    return violation_type == "collision" ||
           violation_type == "line_collision" ||
           violation_type == "out_of_map";
}

}  // namespace fastnav_planner
