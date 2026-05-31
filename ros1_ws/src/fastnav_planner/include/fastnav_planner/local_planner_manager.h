#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <fastnav_mapping/voxel_map.h>

#include "fastnav_planner/astar_planner.h"

namespace fastnav_planner
{

// LocalPlannerManager 负责组合 VoxelMap 和 AStarPlanner。
// ROS node 只把 odom、cloud、goal 转交给 manager，具体地图更新和 A* 搜索都在同一进程内完成。
class LocalPlannerManager
{
public:
    void init(ros::NodeHandle& nh, ros::NodeHandle& pnh);

    void updateOdom(const nav_msgs::OdometryConstPtr& msg);
    void updateCloud(const sensor_msgs::PointCloud2ConstPtr& msg);
    bool planToGoal(const Eigen::Vector3d& goal);

    nav_msgs::Path getPathMsg() const;
    std::vector<Eigen::Vector3d> getPath() const;
    void setPath(const std::vector<Eigen::Vector3d>& path);
    sensor_msgs::PointCloud2 getSearchedNodesCloud() const;
    sensor_msgs::PointCloud2 getDebugOccupiedCloud() const;
    sensor_msgs::PointCloud2 getDebugInflatedCloud() const;

    bool hasOdom() const { return has_odom_; }
    bool hasMap() const { return has_map_; }
    bool hasPath() const { return has_path_; }
    bool replanOnCloud() const { return replan_on_cloud_; }
    std::string frameId() const { return frame_id_; }
    std::string lastError() const { return last_error_; }
    std::shared_ptr<fastnav_mapping::VoxelMap> voxelMap() const { return voxel_map_; }

private:
    void loadParameters(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    void updateDebugClouds(const ros::Time& stamp);
    sensor_msgs::PointCloud2 centersToCloud(const std::vector<Eigen::Vector3d>& centers,
                                            const ros::Time& stamp) const;
    Eigen::Vector3d currentPosition() const;

private:
    std::shared_ptr<fastnav_mapping::VoxelMap> voxel_map_;
    std::shared_ptr<AStarPlanner> astar_planner_;

    std::string frame_id_{"odom"};
    bool replan_on_cloud_{false};

    double resolution_{0.2};
    double local_x_size_{20.0};
    double local_y_size_{20.0};
    double local_z_size_{6.0};
    double local_z_min_{-2.0};
    double local_z_max_{4.0};
    double drone_radius_{0.35};
    double safety_margin_{0.15};
    double map_p_hit_{0.70};
    double map_p_miss_{0.45};
    double map_p_min_{0.12};
    double map_p_max_{0.97};
    double map_p_occ_{0.65};
    double map_temporal_decay_log_{0.05};

    AStarPlanner::Config astar_config_;

    bool has_odom_{false};
    bool has_map_{false};
    bool has_path_{false};

    nav_msgs::Odometry current_odom_;
    ros::Time last_cloud_stamp_;
    Eigen::Vector3d last_goal_{0.0, 0.0, 0.0};

    std::vector<Eigen::Vector3d> current_path_;
    sensor_msgs::PointCloud2 debug_occupied_cloud_;
    sensor_msgs::PointCloud2 debug_inflated_cloud_;
    sensor_msgs::PointCloud2 searched_nodes_cloud_;
    std::string last_error_;
};

}  // namespace fastnav_planner
