#include <ros/ros.h>

#include "fastnav_gz_bridge/gz_odom_bridge.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fastnav_gz_odom_bridge_node");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    fastnav_gz_bridge::GzOdomBridge bridge(nh, pnh);

    if (!bridge.start())
    {
        ROS_ERROR("[FastNav][GzOdomBridgeNode] Failed to start bridge.");
        return 1;
    }

    ROS_INFO("[FastNav][GzOdomBridgeNode] Bridge node started.");

    ros::spin();

    return 0;
}
