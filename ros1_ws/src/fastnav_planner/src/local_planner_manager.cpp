#include "fastnav_planner/local_planner_manager.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace fastnav_planner
{

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

    ROS_INFO("[FastNav][LocalPlannerManager] Initialized. resolution=%.3f, frame=%s",
             resolution_, frame_id_.c_str());
}

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

    pnh.param<std::string>("frame_id", frame_id_, frame_id_);
    pnh.param<bool>("replan_on_cloud", replan_on_cloud_, replan_on_cloud_);
}

void LocalPlannerManager::updateOdom(const nav_msgs::OdometryConstPtr& msg)
{
    current_odom_ = *msg;
    has_odom_ = true;
}

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
    const bool success = astar_planner_->plan(start, goal, current_path_);
    searched_nodes_cloud_ = centersToCloud(astar_planner_->searchedNodes(), ros::Time::now());

    if (!success)
    {
        last_error_ = astar_planner_->lastError();
        return false;
    }

    has_path_ = true;
    last_error_.clear();
    return true;
}

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

std::vector<Eigen::Vector3d> LocalPlannerManager::getPath() const
{
    return current_path_;
}

void LocalPlannerManager::setPath(const std::vector<Eigen::Vector3d>& path)
{
    current_path_ = path;
    has_path_ = !current_path_.empty();
}

sensor_msgs::PointCloud2 LocalPlannerManager::getSearchedNodesCloud() const
{
    return searched_nodes_cloud_;
}

sensor_msgs::PointCloud2 LocalPlannerManager::getDebugOccupiedCloud() const
{
    return debug_occupied_cloud_;
}

sensor_msgs::PointCloud2 LocalPlannerManager::getDebugInflatedCloud() const
{
    return debug_inflated_cloud_;
}

void LocalPlannerManager::updateDebugClouds(const ros::Time& stamp)
{
    debug_occupied_cloud_ = centersToCloud(voxel_map_->getOccupiedVoxelCenters(), stamp);
    debug_inflated_cloud_ = centersToCloud(voxel_map_->getInflatedVoxelCenters(true), stamp);
}

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

Eigen::Vector3d LocalPlannerManager::currentPosition() const
{
    return Eigen::Vector3d(current_odom_.pose.pose.position.x,
                           current_odom_.pose.pose.position.y,
                           current_odom_.pose.pose.position.z);
}

}  // namespace fastnav_planner
