#include "fastnav_gz_bridge/gz_odom_bridge.h"

#include <algorithm>

namespace fastnav_gz_bridge
{

GzOdomBridge::GzOdomBridge(ros::NodeHandle& nh,
                           ros::NodeHandle& pnh)
    : nh_(nh),
      pnh_(pnh),
      gz_topic_("/model/x500_mid360_0/odometry_with_covariance"),
      ros_topic_("/fastnav/state/odom"),
      frame_id_("odom"),
      child_frame_id_("base_link"),
      use_sim_time_stamp_(false)
{
    loadParameters();

    ros_odom_pub_ = nh_.advertise<nav_msgs::Odometry>(ros_topic_, 10);

    ROS_INFO("[FastNav][GzOdomBridge] Initialized.");
    ROS_INFO("[FastNav][GzOdomBridge] gz topic: %s", gz_topic_.c_str());
    ROS_INFO("[FastNav][GzOdomBridge] ros topic: %s", ros_topic_.c_str());
    ROS_INFO("[FastNav][GzOdomBridge] frame_id: %s", frame_id_.c_str());
    ROS_INFO("[FastNav][GzOdomBridge] child_frame_id: %s", child_frame_id_.c_str());
}

void GzOdomBridge::loadParameters()
{
    pnh_.param<std::string>("odom/gz_topic",
                            gz_topic_,
                            gz_topic_);
    pnh_.param<std::string>("gz_topic",
                            gz_topic_,
                            gz_topic_);

    pnh_.param<std::string>("odom/ros_topic",
                            ros_topic_,
                            ros_topic_);
    pnh_.param<std::string>("ros_topic",
                            ros_topic_,
                            ros_topic_);

    pnh_.param<std::string>("odom/frame_id",
                            frame_id_,
                            frame_id_);
    pnh_.param<std::string>("frame_id",
                            frame_id_,
                            frame_id_);

    pnh_.param<std::string>("odom/child_frame_id",
                            child_frame_id_,
                            child_frame_id_);
    pnh_.param<std::string>("child_frame_id",
                            child_frame_id_,
                            child_frame_id_);

    pnh_.param<bool>("odom/use_sim_time_stamp",
                     use_sim_time_stamp_,
                     use_sim_time_stamp_);
    pnh_.param<bool>("use_sim_time_stamp",
                     use_sim_time_stamp_,
                     use_sim_time_stamp_);
}

bool GzOdomBridge::start()
{
    const bool subscribed = gz_node_.Subscribe(
        gz_topic_,
        &GzOdomBridge::odomCallback,
        this);

    if (!subscribed)
    {
        ROS_ERROR("[FastNav][GzOdomBridge] Failed to subscribe gz topic: %s",
                  gz_topic_.c_str());
        return false;
    }

    ROS_INFO("[FastNav][GzOdomBridge] Subscribed gz topic successfully.");
    return true;
}

void GzOdomBridge::odomCallback(
    const gz::msgs::OdometryWithCovariance& gz_msg)
{
    const nav_msgs::Odometry ros_msg = convertToRosOdom(gz_msg);
    ros_odom_pub_.publish(ros_msg);

    ROS_INFO_THROTTLE(1.0,
                      "[FastNav][GzOdomBridge] Bridging odom: p=(%.3f, %.3f, %.3f)",
                      ros_msg.pose.pose.position.x,
                      ros_msg.pose.pose.position.y,
                      ros_msg.pose.pose.position.z);
}

nav_msgs::Odometry GzOdomBridge::convertToRosOdom(
    const gz::msgs::OdometryWithCovariance& gz_msg) const
{
    nav_msgs::Odometry ros_msg;

    ros_msg.header.stamp = resolveStamp(gz_msg.header());
    ros_msg.header.frame_id = frame_id_;
    ros_msg.child_frame_id = child_frame_id_;

    ros_msg.pose.pose.orientation.w = 1.0;

    if (gz_msg.has_pose_with_covariance())
    {
        const auto& gz_pose_with_covariance = gz_msg.pose_with_covariance();
        const auto& gz_pose = gz_pose_with_covariance.pose();

        if (gz_pose.has_position())
        {
            const auto& gz_position = gz_pose.position();
            ros_msg.pose.pose.position.x = gz_position.x();
            ros_msg.pose.pose.position.y = gz_position.y();
            ros_msg.pose.pose.position.z = gz_position.z();
        }

        if (gz_pose.has_orientation())
        {
            const auto& gz_orientation = gz_pose.orientation();
            ros_msg.pose.pose.orientation.x = gz_orientation.x();
            ros_msg.pose.pose.orientation.y = gz_orientation.y();
            ros_msg.pose.pose.orientation.z = gz_orientation.z();
            ros_msg.pose.pose.orientation.w = gz_orientation.w();
        }

        if (gz_pose_with_covariance.has_covariance())
        {
            copyCovariance(gz_pose_with_covariance.covariance(),
                           ros_msg.pose.covariance);
        }
    }

    if (gz_msg.has_twist_with_covariance())
    {
        const auto& gz_twist_with_covariance = gz_msg.twist_with_covariance();
        const auto& gz_twist = gz_twist_with_covariance.twist();

        if (gz_twist.has_linear())
        {
            const auto& gz_linear = gz_twist.linear();
            ros_msg.twist.twist.linear.x = gz_linear.x();
            ros_msg.twist.twist.linear.y = gz_linear.y();
            ros_msg.twist.twist.linear.z = gz_linear.z();
        }

        if (gz_twist.has_angular())
        {
            const auto& gz_angular = gz_twist.angular();
            ros_msg.twist.twist.angular.x = gz_angular.x();
            ros_msg.twist.twist.angular.y = gz_angular.y();
            ros_msg.twist.twist.angular.z = gz_angular.z();
        }

        if (gz_twist_with_covariance.has_covariance())
        {
            copyCovariance(gz_twist_with_covariance.covariance(),
                           ros_msg.twist.covariance);
        }
    }

    return ros_msg;
}

ros::Time GzOdomBridge::resolveStamp(const gz::msgs::Header& gz_header) const
{
    if (use_sim_time_stamp_ && gz_header.has_stamp())
    {
        const auto& gz_stamp = gz_header.stamp();
        if (gz_stamp.sec() >= 0 && gz_stamp.nsec() >= 0)
        {
            return ros::Time(static_cast<uint32_t>(gz_stamp.sec()),
                             static_cast<uint32_t>(gz_stamp.nsec()));
        }
    }

    return ros::Time::now();
}

void GzOdomBridge::copyCovariance(
    const gz::msgs::Float_V& gz_covariance,
    boost::array<double, 36>& ros_covariance) const
{
    std::fill(ros_covariance.begin(), ros_covariance.end(), 0.0);

    const int copy_size = std::min<int>(ros_covariance.size(),
                                        gz_covariance.data_size());
    for (int i = 0; i < copy_size; ++i)
    {
        ros_covariance[i] = gz_covariance.data(i);
    }
}

}  // namespace fastnav_gz_bridge
