#pragma once
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>

#include <gz/transport/Node.hh>
#include <gz/msgs/pointcloud_packed.pb.h>

namespace fastnav_gz_bridge
{

class GzPointCloudBridge
{
public:
    GzPointCloudBridge(ros::NodeHandle& nh, ros::NodeHandle& pnh);

    bool start();

private:
    void loadParameters();

    void pointCloudCallback(const gz::msgs::PointCloudPacked& gz_msg);

    sensor_msgs::PointCloud2 convertToRosPointCloud(
        const gz::msgs::PointCloudPacked& gz_msg) const;

    uint8_t convertFieldDatatype(
        gz::msgs::PointCloudPacked::Field::DataType gz_datatype) const;

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Publisher ros_cloud_pub_;

    gz::transport::Node gz_node_;

    std::string gz_topic_;
    std::string ros_topic_;
    std::string frame_id_;

    bool use_sim_time_stamp_;
};

}  // namespace fastnav_gz_bridge

