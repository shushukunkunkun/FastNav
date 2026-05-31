#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>

class FastNavTfBroadcaster
{
public:
    FastNavTfBroadcaster(ros::NodeHandle& nh, ros::NodeHandle& pnh)
        : parent_frame_("odom"),
          child_frame_("base_link"),
          odom_topic_("/fastnav/state/odom")
    {
        pnh.param<std::string>("parent_frame", parent_frame_, parent_frame_);
        pnh.param<std::string>("child_frame", child_frame_, child_frame_);
        pnh.param<std::string>("odom_topic", odom_topic_, odom_topic_);

        odom_sub_ = nh.subscribe<nav_msgs::Odometry>(
            odom_topic_, 20, &FastNavTfBroadcaster::odomCallback, this);

        ROS_INFO("[FastNav][TfBroadcaster] odom topic: %s", odom_topic_.c_str());
        ROS_INFO("[FastNav][TfBroadcaster] publishing TF: %s -> %s",
                 parent_frame_.c_str(), child_frame_.c_str());
    }

private:
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        geometry_msgs::TransformStamped transform;

        transform.header.stamp = msg->header.stamp;
        transform.header.frame_id = parent_frame_;
        transform.child_frame_id = child_frame_;

        transform.transform.translation.x = msg->pose.pose.position.x;
        transform.transform.translation.y = msg->pose.pose.position.y;
        transform.transform.translation.z = msg->pose.pose.position.z;
        transform.transform.rotation = msg->pose.pose.orientation;

        tf_broadcaster_.sendTransform(transform);
    }

private:
    ros::Subscriber odom_sub_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    std::string parent_frame_;
    std::string child_frame_;
    std::string odom_topic_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fastnav_tf_broadcaster_node");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    FastNavTfBroadcaster broadcaster(nh, pnh);

    ros::spin();

    return 0;
}
