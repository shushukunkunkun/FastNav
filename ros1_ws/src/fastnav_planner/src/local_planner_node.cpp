#include <ros/ros.h>

#include "fastnav_planner/fsm/planner_fsm.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fastnav_local_planner_node");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    fastnav_planner::PlannerFSM planner_fsm;
    planner_fsm.init(nh, pnh);

    ros::spin();
    return 0;
}
