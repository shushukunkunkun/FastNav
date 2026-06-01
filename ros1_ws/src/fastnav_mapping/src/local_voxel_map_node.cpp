#include <string>
#include <vector>

#include <Eigen/Core>
#include <nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>

#include "fastnav_mapping/voxel_map.h"

namespace fastnav_mapping
{

// local_voxel_map_node 是 perception 和 planner 之间的地图 adapter。
// 它输入 odom 点云 $P_f$，输出 occupied 和 inflated 两种体素点云，planner 后续可以直接查询 VoxelMap。
class LocalVoxelMapNode
{
public:
    LocalVoxelMapNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
        : nh_(nh),
          pnh_(pnh)
    {
        loadParameters();

        voxel_map_.init(resolution_,
                        local_x_size_,
                        local_y_size_,
                        local_z_size_,
                        local_z_min_,
                        local_z_max_,
                        drone_radius_,
                        safety_margin_);
        voxel_map_.setOccupancyUpdateParams(map_p_hit_,
                                             map_p_miss_,
                                             map_p_min_,
                                             map_p_max_,
                                             map_p_occ_,
                                             map_temporal_decay_log_);

        cloud_sub_ = nh_.subscribe(input_cloud_topic_, 2,
                                   &LocalVoxelMapNode::cloudCallback,
                                   this);
        odom_sub_ = nh_.subscribe(odom_topic_, 20,
                                  &LocalVoxelMapNode::odomCallback,
                                  this);

        occupied_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(occupied_cloud_topic_, 2);
        inflated_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(inflated_cloud_topic_, 2);
        if (publish_voxel_markers_)
        {
            occupied_voxel_pub_ = nh_.advertise<visualization_msgs::Marker>(occupied_voxel_topic_, 2);
            inflated_voxel_pub_ = nh_.advertise<visualization_msgs::Marker>(inflated_voxel_topic_, 2);
        }

        if (publish_rate_ > 0.0)
        {
            publish_timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_),
                                             &LocalVoxelMapNode::publishTimerCallback,
                                             this);
        }

        ROS_INFO("[FastNav][LocalVoxelMap] input cloud: %s", input_cloud_topic_.c_str());
        ROS_INFO("[FastNav][LocalVoxelMap] odom topic: %s", odom_topic_.c_str());
        ROS_INFO("[FastNav][LocalVoxelMap] resolution: %.3f, inflation_radius: %.3f",
                 resolution_, voxel_map_.inflationRadius());
    }

private:
    void loadParameters()
    {
        // 读取全局 yaml 参数 $/local_voxel_map/*$，保持仿真和真机共用同一个 mapping 节点。
        nh_.param<std::string>("/local_voxel_map/input_cloud_topic", input_cloud_topic_, input_cloud_topic_);
        nh_.param<std::string>("/local_voxel_map/odom_topic", odom_topic_, odom_topic_);
        nh_.param<std::string>("/local_voxel_map/occupied_cloud_topic", occupied_cloud_topic_, occupied_cloud_topic_);
        nh_.param<std::string>("/local_voxel_map/inflated_cloud_topic", inflated_cloud_topic_, inflated_cloud_topic_);
        nh_.param<std::string>("/local_voxel_map/occupied_voxel_topic", occupied_voxel_topic_, occupied_voxel_topic_);
        nh_.param<std::string>("/local_voxel_map/inflated_voxel_topic", inflated_voxel_topic_, inflated_voxel_topic_);
        nh_.param<std::string>("/local_voxel_map/frame_id", frame_id_, frame_id_);
        nh_.param<double>("/local_voxel_map/resolution", resolution_, resolution_);
        nh_.param<double>("/local_voxel_map/local_x_size", local_x_size_, local_x_size_);
        nh_.param<double>("/local_voxel_map/local_y_size", local_y_size_, local_y_size_);
        nh_.param<double>("/local_voxel_map/local_z_size", local_z_size_, local_z_size_);
        nh_.param<double>("/local_voxel_map/local_z_min", local_z_min_, local_z_min_);
        nh_.param<double>("/local_voxel_map/local_z_max", local_z_max_, local_z_max_);
        nh_.param<double>("/local_voxel_map/drone_radius", drone_radius_, drone_radius_);
        nh_.param<double>("/local_voxel_map/safety_margin", safety_margin_, safety_margin_);
        nh_.param<double>("/local_voxel_map/publish_rate", publish_rate_, publish_rate_);
        nh_.param<bool>("/local_voxel_map/publish_voxel_markers", publish_voxel_markers_, publish_voxel_markers_);
        nh_.param<double>("/local_voxel_map/p_hit", map_p_hit_, map_p_hit_);
        nh_.param<double>("/local_voxel_map/p_miss", map_p_miss_, map_p_miss_);
        nh_.param<double>("/local_voxel_map/p_min", map_p_min_, map_p_min_);
        nh_.param<double>("/local_voxel_map/p_max", map_p_max_, map_p_max_);
        nh_.param<double>("/local_voxel_map/p_occ", map_p_occ_, map_p_occ_);
        nh_.param<double>("/local_voxel_map/temporal_decay_log", map_temporal_decay_log_, map_temporal_decay_log_);

        // 私有参数提供局部覆盖入口，方便未来 launch 多地图节点或实验不同 $resolution$。
        pnh_.param<std::string>("input_cloud_topic", input_cloud_topic_, input_cloud_topic_);
        pnh_.param<std::string>("odom_topic", odom_topic_, odom_topic_);
        pnh_.param<std::string>("occupied_cloud_topic", occupied_cloud_topic_, occupied_cloud_topic_);
        pnh_.param<std::string>("inflated_cloud_topic", inflated_cloud_topic_, inflated_cloud_topic_);
        pnh_.param<std::string>("occupied_voxel_topic", occupied_voxel_topic_, occupied_voxel_topic_);
        pnh_.param<std::string>("inflated_voxel_topic", inflated_voxel_topic_, inflated_voxel_topic_);
        pnh_.param<std::string>("frame_id", frame_id_, frame_id_);
        pnh_.param<double>("resolution", resolution_, resolution_);
        pnh_.param<double>("local_x_size", local_x_size_, local_x_size_);
        pnh_.param<double>("local_y_size", local_y_size_, local_y_size_);
        pnh_.param<double>("local_z_size", local_z_size_, local_z_size_);
        pnh_.param<double>("local_z_min", local_z_min_, local_z_min_);
        pnh_.param<double>("local_z_max", local_z_max_, local_z_max_);
        pnh_.param<double>("drone_radius", drone_radius_, drone_radius_);
        pnh_.param<double>("safety_margin", safety_margin_, safety_margin_);
        pnh_.param<double>("publish_rate", publish_rate_, publish_rate_);
        pnh_.param<bool>("publish_voxel_markers", publish_voxel_markers_, publish_voxel_markers_);
        pnh_.param<double>("p_hit", map_p_hit_, map_p_hit_);
        pnh_.param<double>("p_miss", map_p_miss_, map_p_miss_);
        pnh_.param<double>("p_min", map_p_min_, map_p_min_);
        pnh_.param<double>("p_max", map_p_max_, map_p_max_);
        pnh_.param<double>("p_occ", map_p_occ_, map_p_occ_);
        pnh_.param<double>("temporal_decay_log", map_temporal_decay_log_, map_temporal_decay_log_);
    }

    void odomCallback(const nav_msgs::OdometryConstPtr& msg)
    {
        latest_center_.x() = msg->pose.pose.position.x;
        latest_center_.y() = msg->pose.pose.position.y;
        latest_center_.z() = msg->pose.pose.position.z;
        has_odom_ = true;
    }

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        if (msg->header.frame_id != frame_id_)
        {
            ROS_WARN_THROTTLE(1.0,
                              "[FastNav][LocalVoxelMap] Expected cloud frame %s, got %s. Drop this frame.",
                              frame_id_.c_str(), msg->header.frame_id.c_str());
            return;
        }

        if (!has_odom_)
        {
            ROS_WARN_THROTTLE(1.0,
                              "[FastNav][LocalVoxelMap] Waiting for /fastnav/state/odom before building local map.");
            return;
        }

        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*msg, cloud);

        // 点云回调只负责更新地图缓存，不直接作为一帧地图发布。
        // 地图中心跟随无人机位置 $c$，VoxelMap 会在跨过体素边界时平移内部 buffer。
        // 与 EGO-Planner 类似，地图长期维护 log-odds 缓存；这里不 reset，避免 RViz 中障碍物一帧有一帧无。
        voxel_map_.setMapCenter(latest_center_);
        voxel_map_.decayOccupancy();

        for (const pcl::PointXYZ& point : cloud.points)
        {
            const Eigen::Vector3d pos(point.x, point.y, point.z);
            voxel_map_.setOccupied(pos);
        }

        // occupied 来自真实点云命中，inflated 是按无人机半径和安全裕度扩出的不可通行区域。
        voxel_map_.inflateObstacles();

        const std::vector<Eigen::Vector3d> occupied_centers = voxel_map_.getOccupiedVoxelCenters();
        const std::vector<Eigen::Vector3d> inflated_centers = voxel_map_.getInflatedVoxelCenters(true);

        latest_occupied_msg_ = centersToCloud(occupied_centers, msg->header.stamp);
        latest_inflated_msg_ = centersToCloud(inflated_centers, msg->header.stamp);
        if (publish_voxel_markers_)
        {
            latest_occupied_marker_ = centersToCubeList(occupied_centers,
                                                        msg->header.stamp,
                                                        "occupied_voxels",
                                                        0,
                                                        1.0,
                                                        0.08,
                                                        0.02,
                                                        0.85);
            latest_inflated_marker_ = centersToCubeList(inflated_centers,
                                                        msg->header.stamp,
                                                        "inflated_voxels",
                                                        1,
                                                        0.1,
                                                        0.45,
                                                        1.0,
                                                        0.32);
        }
        has_map_ = true;

        if (publish_rate_ <= 0.0)
        {
            publishCurrentMap();
        }

        ROS_INFO_THROTTLE(1.0,
                          "[FastNav][LocalVoxelMap] cloud=%zu, occupied=%u, inflated=%u",
                          cloud.size(),
                          latest_occupied_msg_.width * latest_occupied_msg_.height,
                          latest_inflated_msg_.width * latest_inflated_msg_.height);
    }

    sensor_msgs::PointCloud2 centersToCloud(const std::vector<Eigen::Vector3d>& centers,
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

    visualization_msgs::Marker centersToCubeList(const std::vector<Eigen::Vector3d>& centers,
                                                 const ros::Time& stamp,
                                                 const std::string& ns,
                                                 int id,
                                                 double r,
                                                 double g,
                                                 double b,
                                                 double a) const
    {
        visualization_msgs::Marker marker;
        marker.header.stamp = stamp;
        marker.header.frame_id = frame_id_;
        marker.ns = ns;
        marker.id = id;
        marker.type = visualization_msgs::Marker::CUBE_LIST;
        marker.action = visualization_msgs::Marker::ADD;

        // CUBE_LIST 中每个点 $p_i$ 是一个体素中心，显示尺寸取 $resolution$，即立方体边长等于体素边长。
        marker.scale.x = resolution_;
        marker.scale.y = resolution_;
        marker.scale.z = resolution_;
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = a;
        marker.pose.orientation.w = 1.0;
        // lifetime 为 $0$ 表示不过期；下一帧同一个 $ns/id$ 的 Marker 会覆盖当前 Marker。
        // 避免定时发布稍有抖动时，RViz 先删除旧 Marker 再添加新 Marker，从而产生闪烁。
        marker.lifetime = ros::Duration(0.0);

        marker.points.reserve(centers.size());
        for (const Eigen::Vector3d& center : centers)
        {
            geometry_msgs::Point point;
            point.x = center.x();
            point.y = center.y();
            point.z = center.z();
            marker.points.push_back(point);
        }

        return marker;
    }

    void publishTimerCallback(const ros::TimerEvent&)
    {
        publishCurrentMap();
    }

    void publishCurrentMap()
    {
        if (!has_map_)
        {
            return;
        }

        occupied_pub_.publish(latest_occupied_msg_);
        inflated_pub_.publish(latest_inflated_msg_);
        if (publish_voxel_markers_)
        {
            occupied_voxel_pub_.publish(latest_occupied_marker_);
            inflated_voxel_pub_.publish(latest_inflated_marker_);
        }
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber cloud_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher occupied_pub_;
    ros::Publisher inflated_pub_;
    ros::Publisher occupied_voxel_pub_;
    ros::Publisher inflated_voxel_pub_;
    ros::Timer publish_timer_;

    VoxelMap voxel_map_;

    std::string input_cloud_topic_{"/fastnav/perception/cloud_filtered"};
    std::string odom_topic_{"/fastnav/state/odom"};
    std::string occupied_cloud_topic_{"/fastnav/map/occupied_cloud"};
    std::string inflated_cloud_topic_{"/fastnav/map/inflated_cloud"};
    std::string occupied_voxel_topic_{"/fastnav/map/occupied_voxels"};
    std::string inflated_voxel_topic_{"/fastnav/map/inflated_voxels"};
    std::string frame_id_{"odom"};

    double resolution_{0.2};
    double local_x_size_{20.0};
    double local_y_size_{20.0};
    double local_z_size_{6.0};
    double local_z_min_{-2.0};
    double local_z_max_{4.0};
    double drone_radius_{0.35};
    double safety_margin_{0.15};
    double publish_rate_{10.0};
    bool publish_voxel_markers_{false};
    double map_p_hit_{0.70};
    double map_p_miss_{0.45};
    double map_p_min_{0.12};
    double map_p_max_{0.97};
    double map_p_occ_{0.65};
    double map_temporal_decay_log_{0.05};

    bool has_odom_{false};
    bool has_map_{false};
    Eigen::Vector3d latest_center_{0.0, 0.0, 0.0};
    sensor_msgs::PointCloud2 latest_occupied_msg_;
    sensor_msgs::PointCloud2 latest_inflated_msg_;
    visualization_msgs::Marker latest_occupied_marker_;
    visualization_msgs::Marker latest_inflated_marker_;
};

}  // namespace fastnav_mapping

int main(int argc, char** argv)
{
    ros::init(argc, argv, "local_voxel_map_node");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    fastnav_mapping::LocalVoxelMapNode node(nh, pnh);

    ros::spin();
    return 0;
}
