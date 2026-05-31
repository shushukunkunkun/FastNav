#include "fastnav_gz_bridge/gz_pointcloud_bridge.h"

#include <cstring>

namespace fastnav_gz_bridge
{

GzPointCloudBridge::GzPointCloudBridge(ros::NodeHandle& nh,
                                       ros::NodeHandle& pnh)
    : nh_(nh),
      pnh_(pnh),
      gz_topic_("/lidar_points/points"),
      ros_topic_("/fastnav/lidar/points"),
      frame_id_("mid360_link"),
      use_sim_time_stamp_(false)
{
    loadParameters();

    ros_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
        ros_topic_, 10);

    ROS_INFO("[FastNav][GzPointCloudBridge] Initialized.");
    ROS_INFO("[FastNav][GzPointCloudBridge] gz topic: %s",
             gz_topic_.c_str());
    ROS_INFO("[FastNav][GzPointCloudBridge] ros topic: %s",
             ros_topic_.c_str());
    ROS_INFO("[FastNav][GzPointCloudBridge] frame_id: %s",
             frame_id_.c_str());
}

void GzPointCloudBridge::loadParameters()
{
    pnh_.param<std::string>("pointcloud/gz_topic",
                            gz_topic_,
                            gz_topic_);
    pnh_.param<std::string>("gz_topic",
                            gz_topic_,
                            gz_topic_);

    pnh_.param<std::string>("pointcloud/ros_topic",
                            ros_topic_,
                            ros_topic_);
    pnh_.param<std::string>("ros_topic",
                            ros_topic_,
                            ros_topic_);

    pnh_.param<std::string>("pointcloud/frame_id",
                            frame_id_,
                            frame_id_);
    pnh_.param<std::string>("frame_id",
                            frame_id_,
                            frame_id_);

    pnh_.param<bool>("pointcloud/use_sim_time_stamp",
                     use_sim_time_stamp_,
                     use_sim_time_stamp_);
    pnh_.param<bool>("use_sim_time_stamp",
                     use_sim_time_stamp_,
                     use_sim_time_stamp_);
}

bool GzPointCloudBridge::start()
{
    const bool subscribed = gz_node_.Subscribe(
        gz_topic_,
        &GzPointCloudBridge::pointCloudCallback,
        this);

    if (!subscribed)
    {
        ROS_ERROR("[FastNav][GzPointCloudBridge] Failed to subscribe gz topic: %s",
                  gz_topic_.c_str());
        return false;
    }

    ROS_INFO("[FastNav][GzPointCloudBridge] Subscribed gz topic successfully.");
    return true;
}

void GzPointCloudBridge::pointCloudCallback(
    const gz::msgs::PointCloudPacked& gz_msg)
{
    const sensor_msgs::PointCloud2 ros_msg = convertToRosPointCloud(gz_msg);
    ros_cloud_pub_.publish(ros_msg);

    ROS_INFO_THROTTLE(1.0,
                      "[FastNav][GzPointCloudBridge] Bridging point cloud: width=%u, height=%u, point_step=%u, data=%zu bytes",
                      ros_msg.width,
                      ros_msg.height,
                      ros_msg.point_step,
                      ros_msg.data.size());
}

sensor_msgs::PointCloud2 GzPointCloudBridge::convertToRosPointCloud(
    const gz::msgs::PointCloudPacked& gz_msg) const
{
    sensor_msgs::PointCloud2 ros_msg;

    /*
     * Gazebo PointCloudPacked has its own header, but for the first version
     * we use ros::Time::now(). Later, if needed, we can parse Gazebo sim time.
     */
    ros_msg.header.stamp = ros::Time::now();
    ros_msg.header.frame_id = frame_id_;

    ros_msg.height = gz_msg.height();
    ros_msg.width = gz_msg.width();

    ros_msg.fields.clear();
    ros_msg.fields.reserve(gz_msg.field_size());

    for (int i = 0; i < gz_msg.field_size(); ++i)
    {
        const auto& gz_field = gz_msg.field(i);

        sensor_msgs::PointField ros_field;
        ros_field.name = gz_field.name();
        ros_field.offset = gz_field.offset();
        ros_field.datatype = convertFieldDatatype(gz_field.datatype());
        ros_field.count = gz_field.count();

        if (ros_field.datatype == 0)
        {
            ROS_WARN_THROTTLE(1.0,
                              "[FastNav][GzPointCloudBridge] Unsupported point field datatype. field: %s",
                              ros_field.name.c_str());
            continue;
        }

        ros_msg.fields.push_back(ros_field);
    }

    ros_msg.is_bigendian = gz_msg.is_bigendian();
    ros_msg.point_step = gz_msg.point_step();
    ros_msg.row_step = gz_msg.row_step();
    ros_msg.is_dense = gz_msg.is_dense();

    const std::string& gz_data = gz_msg.data();

    ros_msg.data.resize(gz_data.size());
    if (!gz_data.empty())
    {
        std::memcpy(ros_msg.data.data(),
                    gz_data.data(),
                    gz_data.size());
    }

    return ros_msg;
}

uint8_t GzPointCloudBridge::convertFieldDatatype(
    gz::msgs::PointCloudPacked::Field::DataType gz_datatype) const
{
    switch (gz_datatype)
    {
    case gz::msgs::PointCloudPacked::Field::INT8:
        return sensor_msgs::PointField::INT8;

    case gz::msgs::PointCloudPacked::Field::UINT8:
        return sensor_msgs::PointField::UINT8;

    case gz::msgs::PointCloudPacked::Field::INT16:
        return sensor_msgs::PointField::INT16;

    case gz::msgs::PointCloudPacked::Field::UINT16:
        return sensor_msgs::PointField::UINT16;

    case gz::msgs::PointCloudPacked::Field::INT32:
        return sensor_msgs::PointField::INT32;

    case gz::msgs::PointCloudPacked::Field::UINT32:
        return sensor_msgs::PointField::UINT32;

    case gz::msgs::PointCloudPacked::Field::FLOAT32:
        return sensor_msgs::PointField::FLOAT32;

    case gz::msgs::PointCloudPacked::Field::FLOAT64:
        return sensor_msgs::PointField::FLOAT64;

    default:
        return 0;
    }
}

}  // namespace fastnav_gz_bridge
