#include <ros/ros.h>

#include "fastnav_control/px4_offboard_interface.h"
#include "fastnav_control/control_fsm.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fastnav_offboard_control_node");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    double control_rate = 20.0;
    pnh.param<double>("control/rate", control_rate, 20.0);

    ROS_INFO("[FastNav][OffboardControlNode] Node started.");
    ROS_INFO("[FastNav][OffboardControlNode] Control rate: %.2f Hz", control_rate);

    fastnav_control::Px4OffboardInterface px4_interface(nh, pnh);
    fastnav_control::ControlFSM control_fsm(nh, pnh, px4_interface);

    ros::Rate rate(control_rate);

    while (ros::ok())
    {
        control_fsm.runOnce();

        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("[FastNav][OffboardControlNode] Node shutdown.");

    return 0;
}