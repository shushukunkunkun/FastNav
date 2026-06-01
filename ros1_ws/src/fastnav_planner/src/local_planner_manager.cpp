/*
 * local_planner_manager.cpp
 *
 * 本文件实现 LocalPlannerManager 类。
 * LocalPlannerManager 是 PlannerFSM 下方的算法组织层，不直接订阅 ROS topic，也不发布 ROS topic。
 * 它的工作是把“状态机事件”转换为“算法对象之间的内存调用”：
 * 1. 持有 fastnav_mapping::VoxelMap，用 filtered cloud 更新 planner 内部局部体素地图；
 * 2. 持有 AStarPlanner，直接查询 VoxelMap 的 inflated occupancy 做 3D A* 搜索；
 * 3. 持有 PathOptimizer，对 A* 原始路径做 shortcut 等轻量优化；
 * 4. 维护 global_data_ 和 local_data_，用 B-spline 保存全局参考轨迹和当前接受的局部轨迹；
 * 5. 缓存 current_odom_、last_goal_、current_path_、debug cloud 和 searched nodes；
 * 6. 向 PlannerFSM 提供 getPathMsg()、getDebugInflatedCloud() 等结果接口。
 *
 * 架构上，FSM 负责“什么时候规划、什么时候重规划”，manager 负责“如何更新地图和生成路径”。
 * 这样 planner 的高频碰撞查询通过共享指针在同一进程内完成，不需要 ROS service / topic 往返。
 */

#include "fastnav_planner/local_planner_manager.h"

#include <algorithm>
#include <cmath>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace fastnav_planner
{

// 初始化 manager 内部算法对象：读取参数，创建 VoxelMap、AStarPlanner 和 PathOptimizer，并完成指针绑定。
void LocalPlannerManager::init(ros::NodeHandle& nh, ros::NodeHandle& pnh)
{
    loadParameters(nh, pnh);

    voxel_map_ = std::make_shared<fastnav_mapping::VoxelMap>();
    voxel_map_->init(resolution_,
                     local_x_size_,
                     local_y_size_,
                     local_z_size_,
                     local_z_min_,
                     local_z_max_,
                     drone_radius_,
                     safety_margin_);
    // planner 内部 VoxelMap 不再每帧完全重建，而是像 EGO-Planner 的 GridMap 一样长期持有局部缓存。
    // 点云命中用 log-odds 累计 $l_i <- clamp(l_i + log(p_hit/(1-p_hit)), l_min, l_max)$；
    // 当前版本暂不做 raycasting free-space，因此用 temporal_decay_log 把旧障碍缓慢拉回未知 $l=0$。
    voxel_map_->setOccupancyUpdateParams(map_p_hit_,
                                         map_p_miss_,
                                         map_p_min_,
                                         map_p_max_,
                                         map_p_occ_,
                                         map_temporal_decay_log_);

    astar_planner_ = std::make_shared<AStarPlanner>();
    astar_planner_->setMap(voxel_map_);
    astar_planner_->setConfig(astar_config_);

    path_optimizer_ = std::make_shared<PathOptimizer>();
    path_optimizer_->setConfig(optimizer_config_);

    global_data_.global_duration_ = 0.0;
    global_data_.global_start_time_ = ros::Time(0);
    global_data_.local_start_time_ = -1.0;
    global_data_.local_end_time_ = -1.0;
    global_data_.time_increase_ = 0.0;
    global_data_.last_time_inc_ = 0.0;
    global_data_.last_progress_time_ = 0.0;

    local_data_.traj_id_ = 0;
    local_data_.duration_ = 0.0;
    local_data_.start_time_ = ros::Time(0);
    local_data_.start_pos_.setZero();

    ROS_INFO("[FastNav][LocalPlannerManager] Initialized. resolution=%.3f, frame=%s",
             resolution_, frame_id_.c_str());
}

// 读取局部地图、A*、优化器参数；全局 yaml 提供默认值，私有参数提供节点级覆盖入口。
void LocalPlannerManager::loadParameters(ros::NodeHandle& nh, ros::NodeHandle& pnh)
{
    nh.param<std::string>("/local_planner/frame_id", frame_id_, frame_id_);
    nh.param<bool>("/local_planner/replan_on_cloud", replan_on_cloud_, replan_on_cloud_);

    nh.param<double>("/local_planner/map/resolution", resolution_, resolution_);
    nh.param<double>("/local_planner/map/local_x_size", local_x_size_, local_x_size_);
    nh.param<double>("/local_planner/map/local_y_size", local_y_size_, local_y_size_);
    nh.param<double>("/local_planner/map/local_z_size", local_z_size_, local_z_size_);
    nh.param<double>("/local_planner/map/local_z_min", local_z_min_, local_z_min_);
    nh.param<double>("/local_planner/map/local_z_max", local_z_max_, local_z_max_);
    nh.param<double>("/local_planner/map/drone_radius", drone_radius_, drone_radius_);
    nh.param<double>("/local_planner/map/safety_margin", safety_margin_, safety_margin_);
    nh.param<double>("/local_planner/map/p_hit", map_p_hit_, map_p_hit_);
    nh.param<double>("/local_planner/map/p_miss", map_p_miss_, map_p_miss_);
    nh.param<double>("/local_planner/map/p_min", map_p_min_, map_p_min_);
    nh.param<double>("/local_planner/map/p_max", map_p_max_, map_p_max_);
    nh.param<double>("/local_planner/map/p_occ", map_p_occ_, map_p_occ_);
    nh.param<double>("/local_planner/map/temporal_decay_log", map_temporal_decay_log_, map_temporal_decay_log_);

    nh.param<bool>("/local_planner/astar/allow_diagonal", astar_config_.allow_diagonal, astar_config_.allow_diagonal);
    nh.param<bool>("/local_planner/astar/check_line_collision", astar_config_.check_line_collision, astar_config_.check_line_collision);
    nh.param<double>("/local_planner/astar/heuristic_weight", astar_config_.heuristic_weight, astar_config_.heuristic_weight);
    nh.param<double>("/local_planner/astar/line_check_step", astar_config_.line_check_step, astar_config_.line_check_step);
    nh.param<int>("/local_planner/astar/max_search_nodes", astar_config_.max_search_nodes, astar_config_.max_search_nodes);
    nh.param<bool>("/local_planner/optimizer/enable", optimizer_config_.enable, optimizer_config_.enable);
    nh.param<bool>("/local_planner/optimizer/shortcut", optimizer_config_.shortcut, optimizer_config_.shortcut);
    nh.param<double>("/local_planner/optimizer/line_check_step", optimizer_config_.line_check_step, optimizer_config_.line_check_step);
    nh.param<int>("/local_planner/trajectory/bspline_degree", bspline_degree_, bspline_degree_);
    nh.param<double>("/local_planner/trajectory/nominal_velocity", traj_nominal_vel_, traj_nominal_vel_);

    pnh.param<std::string>("frame_id", frame_id_, frame_id_);
    pnh.param<bool>("replan_on_cloud", replan_on_cloud_, replan_on_cloud_);
    pnh.param<bool>("optimizer/enable", optimizer_config_.enable, optimizer_config_.enable);
    pnh.param<bool>("optimizer/shortcut", optimizer_config_.shortcut, optimizer_config_.shortcut);
    pnh.param<double>("optimizer/line_check_step", optimizer_config_.line_check_step, optimizer_config_.line_check_step);
    pnh.param<int>("trajectory/bspline_degree", bspline_degree_, bspline_degree_);
    pnh.param<double>("trajectory/nominal_velocity", traj_nominal_vel_, traj_nominal_vel_);

    bspline_degree_ = std::clamp(bspline_degree_, 1, 5);
    traj_nominal_vel_ = std::max(0.1, traj_nominal_vel_);
}

// 更新当前无人机状态；current_odom_ 后续用于地图中心 $c$ 和 A* 搜索起点。
void LocalPlannerManager::updateOdom(const nav_msgs::OdometryConstPtr& msg)
{
    current_odom_ = *msg;
    has_odom_ = true;
}

// 用最新 filtered cloud 更新 planner 内部 VoxelMap，并刷新 occupied / inflated debug cloud。
void LocalPlannerManager::updateCloud(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    if (!has_odom_)
    {
        last_error_ = "Waiting for odom before updating planner map.";
        return;
    }

    if (msg->header.frame_id != frame_id_)
    {
        last_error_ = "Cloud frame is not planner frame.";
        ROS_WARN_THROTTLE(1.0,
                          "[FastNav][LocalPlannerManager] Expected cloud frame %s, got %s.",
                          frame_id_.c_str(), msg->header.frame_id.c_str());
        return;
    }

    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);

    // 局部地图中心跟随无人机位置 $c$ 滑动，VoxelMap 会按整格偏移搬运旧缓存。
    // 因此这里不再 reset；旧障碍若持续被看见会继续累积，没被看见则通过 $l_i -> 0$ 的衰减逐渐遗忘。
    voxel_map_->setMapCenter(currentPosition());
    voxel_map_->decayOccupancy();

    for (const pcl::PointXYZ& point : cloud.points)
    {
        voxel_map_->setOccupied(Eigen::Vector3d(point.x, point.y, point.z));
    }
    voxel_map_->inflateObstacles();

    last_cloud_stamp_ = msg->header.stamp;
    has_map_ = true;
    updateDebugClouds(msg->header.stamp);
}

// 从当前无人机位置规划到目标点：先运行 A*，再调用 PathOptimizer 进行路径简化。
bool LocalPlannerManager::planToGoal(const Eigen::Vector3d& goal)
{
    last_goal_ = goal;
    current_path_.clear();
    has_path_ = false;

    if (!has_odom_)
    {
        last_error_ = "No odom available.";
        return false;
    }
    if (!has_map_)
    {
        last_error_ = "No local map available.";
        return false;
    }

    const Eigen::Vector3d start = currentPosition();
    std::vector<Eigen::Vector3d> raw_path;
    const bool success = astar_planner_->plan(start, goal, raw_path);
    searched_nodes_cloud_ = centersToCloud(astar_planner_->searchedNodes(), ros::Time::now());

    if (!success)
    {
        last_error_ = astar_planner_->lastError();
        return false;
    }

    std::vector<Eigen::Vector3d> optimized_path;
    if (path_optimizer_ && path_optimizer_->optimize(raw_path, voxel_map_, optimized_path))
    {
        current_path_ = optimized_path;
    }
    else
    {
        if (path_optimizer_)
        {
            ROS_WARN_THROTTLE(1.0,
                              "[FastNav][LocalPlannerManager] Path optimization failed: %s. Use raw A* path.",
                              path_optimizer_->lastError().c_str());
        }
        current_path_ = raw_path;
    }

    const ros::Time time_now = ros::Time::now();
    // raw_path 作为任务级参考线写入 global_data_，优化后的 current_path_ 作为当前可执行局部轨迹写入 local_data_。
    updateGlobalTrajInfo(raw_path, time_now);
    updateTrajInfo(current_path_, time_now);

    has_path_ = true;
    last_error_.clear();
    return true;
}

// 将 current_path_ 打包为 nav_msgs::Path，供 FSM 发布给 RViz 或后续控制模块。
nav_msgs::Path LocalPlannerManager::getPathMsg() const
{
    nav_msgs::Path path_msg;
    path_msg.header.stamp = ros::Time::now();
    path_msg.header.frame_id = frame_id_;

    path_msg.poses.reserve(current_path_.size());
    for (const Eigen::Vector3d& point : current_path_)
    {
        geometry_msgs::PoseStamped pose;
        pose.header = path_msg.header;
        pose.pose.position.x = point.x();
        pose.pose.position.y = point.y();
        pose.pose.position.z = point.z();
        pose.pose.orientation.w = 1.0;
        path_msg.poses.push_back(pose);
    }

    return path_msg;
}

// 返回当前路径的 Eigen 点列，供同进程内其他模块直接使用。
std::vector<Eigen::Vector3d> LocalPlannerManager::getPath() const
{
    return current_path_;
}

// 外部设置当前路径；目前主要为后续控制 / 轨迹优化扩展预留。
void LocalPlannerManager::setPath(const std::vector<Eigen::Vector3d>& path)
{
    current_path_ = path;
    has_path_ = !current_path_.empty();
    if (has_path_)
    {
        updateTrajInfo(current_path_, ros::Time::now());
    }
}

// 返回 A* 搜索过的节点点云，用于 RViz 查看搜索膨胀范围。
sensor_msgs::PointCloud2 LocalPlannerManager::getSearchedNodesCloud() const
{
    return searched_nodes_cloud_;
}

// 返回 planner 内部 VoxelMap 的原始 occupied 体素点云。
sensor_msgs::PointCloud2 LocalPlannerManager::getDebugOccupiedCloud() const
{
    return debug_occupied_cloud_;
}

// 返回 planner 内部 VoxelMap 的 inflated 体素点云；这更接近 A* 实际碰撞查询使用的数据。
sensor_msgs::PointCloud2 LocalPlannerManager::getDebugInflatedCloud() const
{
    return debug_inflated_cloud_;
}

// 检查 current_path_ 的每条线段是否仍然在当前 inflated map 中无碰撞。
bool LocalPlannerManager::isCurrentPathCollisionFree(double step_size) const
{
    if (!has_path_ || current_path_.size() < 2 || !voxel_map_)
    {
        return true;
    }

    for (size_t i = 1; i < current_path_.size(); ++i)
    {
        if (!voxel_map_->isLineFree(current_path_[i - 1], current_path_[i], step_size))
        {
            return false;
        }
    }

    return true;
}

// 从 VoxelMap 读取当前 occupied / inflated 体素中心，并转换为 debug PointCloud2 缓存。
void LocalPlannerManager::updateDebugClouds(const ros::Time& stamp)
{
    debug_occupied_cloud_ = centersToCloud(voxel_map_->getOccupiedVoxelCenters(), stamp);
    debug_inflated_cloud_ = centersToCloud(voxel_map_->getInflatedVoxelCenters(true), stamp);
}

// 将体素中心点或搜索节点从 Eigen 点列转换为 sensor_msgs::PointCloud2。
sensor_msgs::PointCloud2 LocalPlannerManager::centersToCloud(
    const std::vector<Eigen::Vector3d>& centers,
    const ros::Time& stamp) const
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.reserve(centers.size());
    for (const Eigen::Vector3d& center : centers)
    {
        cloud.push_back(pcl::PointXYZ(center.x(), center.y(), center.z()));
    }
    cloud.width = cloud.size();
    cloud.height = 1;
    cloud.is_dense = true;

    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(cloud, msg);
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id_;
    return msg;
}

void LocalPlannerManager::updateTrajInfo(const std::vector<Eigen::Vector3d>& path,
                                         const ros::Time& time_now)
{
    ego_planner::UniformBspline position_traj;
    if (!buildUniformBsplineFromPath(path, position_traj))
    {
        return;
    }

    local_data_.start_time_ = time_now;
    local_data_.position_traj_ = position_traj;
    local_data_.velocity_traj_ = local_data_.position_traj_.getDerivative();
    local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();
    local_data_.start_pos_ = local_data_.position_traj_.evaluateDeBoorT(0.0);
    local_data_.duration_ = local_data_.position_traj_.getTimeSum();
    local_data_.traj_id_ += 1;
}

void LocalPlannerManager::updateGlobalTrajInfo(const std::vector<Eigen::Vector3d>& path,
                                               const ros::Time& time_now)
{
    if (path.empty())
    {
        return;
    }

    std::vector<Eigen::Vector3d> points = path;
    if (points.size() == 1)
    {
        // PolynomialTraj 至少需要一段轨迹；单点输入时构造一个极短的零长度段，保持接口可用。
        points.push_back(points.front());
    }

    Eigen::VectorXd time(points.size() - 1);
    for (size_t i = 0; i + 1 < points.size(); ++i)
    {
        // 每段时间按 $t_i=\max(\|p_{i+1}-p_i\|/v_{nom}, 10^{-3})$ 估计，保证多项式段时间为正。
        time(static_cast<int>(i)) = std::max((points[i + 1] - points[i]).norm() / traj_nominal_vel_, 1e-3);
    }

    PolynomialTraj global_traj;
    if (points.size() == 2)
    {
        global_traj = PolynomialTraj::one_segment_traj_gen(points.front(),
                                                           Eigen::Vector3d::Zero(),
                                                           Eigen::Vector3d::Zero(),
                                                           points.back(),
                                                           Eigen::Vector3d::Zero(),
                                                           Eigen::Vector3d::Zero(),
                                                           time(0));
    }
    else
    {
        Eigen::MatrixXd pos(3, points.size());
        for (size_t i = 0; i < points.size(); ++i)
        {
            pos.col(static_cast<int>(i)) = points[i];
        }

        // EGO 风格的全局轨迹用分段五次多项式表示；中间点速度和加速度由最小 jerk 目标自动求解。
        global_traj = PolynomialTraj::minSnapTraj(pos,
                                                  Eigen::Vector3d::Zero(),
                                                  Eigen::Vector3d::Zero(),
                                                  Eigen::Vector3d::Zero(),
                                                  Eigen::Vector3d::Zero(),
                                                  time);
    }

    global_data_.setGlobalTraj(global_traj, time_now);
}

bool LocalPlannerManager::buildUniformBsplineFromPath(const std::vector<Eigen::Vector3d>& path,
                                                      ego_planner::UniformBspline& traj) const
{
    if (path.empty())
    {
        return false;
    }

    std::vector<Eigen::Vector3d> point_set = path;
    while (point_set.size() <= 3)
    {
        point_set.push_back(point_set.back());
    }

    const double duration = estimatePathDuration(point_set);
    const double ts = std::max(duration / std::max<int>(1, static_cast<int>(point_set.size()) - 1), 1e-3);

    std::vector<Eigen::Vector3d> start_end_derivative;
    start_end_derivative.reserve(4);
    // 当前 A* 输出的是几何路径，还没有动力学边界条件；第一版先使用零首尾速度/加速度约束。
    // parameterizeToBspline() 会求解控制点 $Q$，使三次均匀 B 样条在采样时刻尽量满足 $p(k t_s)=p_k$。
    start_end_derivative.push_back(Eigen::Vector3d::Zero());
    start_end_derivative.push_back(Eigen::Vector3d::Zero());
    start_end_derivative.push_back(Eigen::Vector3d::Zero());
    start_end_derivative.push_back(Eigen::Vector3d::Zero());

    Eigen::MatrixXd ctrl_pts;
    ego_planner::UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivative, ctrl_pts);
    if (ctrl_pts.cols() == 0)
    {
        return false;
    }

    // EGO 的 parameterizeToBspline() 固定构造三次均匀 B 样条，因此这里使用 degree/order 为 3。
    traj = ego_planner::UniformBspline(ctrl_pts, 3, ts);
    return true;
}

double LocalPlannerManager::estimatePathDuration(const std::vector<Eigen::Vector3d>& path) const
{
    if (path.size() < 2)
    {
        return 1e-3;
    }

    double length = 0.0;
    for (size_t i = 1; i < path.size(); ++i)
    {
        length += (path[i] - path[i - 1]).norm();
    }
    return std::max(length / traj_nominal_vel_, 1e-3);
}

// 从 current_odom_ 中提取当前位置向量，作为局部地图中心和规划起点。
Eigen::Vector3d LocalPlannerManager::currentPosition() const
{
    return Eigen::Vector3d(current_odom_.pose.pose.position.x,
                           current_odom_.pose.pose.position.y,
                           current_odom_.pose.pose.position.z);
}

}  // namespace fastnav_planner
