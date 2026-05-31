#include <ros/ros.h>

#include "fastnav_gz_bridge/gz_pointcloud_bridge.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fastnav_gz_pointcloud_bridge_node");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    fastnav_gz_bridge::GzPointCloudBridge bridge(nh, pnh);

    if (!bridge.start())
    {
        ROS_ERROR("[FastNav][GzPointCloudBridgeNode] Failed to start bridge.");
        return 1;
    }

    ROS_INFO("[FastNav][GzPointCloudBridgeNode] Bridge node started.");

    ros::spin();

    return 0;
}