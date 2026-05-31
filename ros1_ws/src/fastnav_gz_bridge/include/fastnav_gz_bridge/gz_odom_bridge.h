#pragma once

#include <string>

#include <boost/array.hpp>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <gz/msgs/float_v.pb.h>
#include <gz/msgs/header.pb.h>
#include <gz/msgs/odometry_with_covariance.pb.h>
#include <gz/transport/Node.hh>

namespace fastnav_gz_bridge
{

class GzOdomBridge
{
public:
    GzOdomBridge(ros::NodeHandle& nh, ros::NodeHandle& pnh);

    bool start();

private:
    void loadParameters();

    void odomCallback(const gz::msgs::OdometryWithCovariance& gz_msg);

    nav_msgs::Odometry convertToRosOdom(
        const gz::msgs::OdometryWithCovariance& gz_msg) const;

    ros::Time resolveStamp(const gz::msgs::Header& gz_header) const;

    void copyCovariance(const gz::msgs::Float_V& gz_covariance,
                        boost::array<double, 36>& ros_covariance) const;

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Publisher ros_odom_pub_;

    gz::transport::Node gz_node_;

    std::string gz_topic_;
    std::string ros_topic_;
    std::string frame_id_;
    std::string child_frame_id_;

    bool use_sim_time_stamp_;
};

}  // namespace fastnav_gz_bridge
