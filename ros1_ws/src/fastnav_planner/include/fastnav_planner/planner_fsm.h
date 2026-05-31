#pragma once

#include <memory>
#include <string>

#include <Eigen/Core>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "fastnav_planner/local_planner_manager.h"
#include "fastnav_planner/path_optimizer.h"

namespace fastnav_planner
{

// PlannerFSM 是 FastNav planner 的显式状态机。
// 它只负责状态转换、ROS 输入输出和调用 manager / optimizer，不构造控制量。
class PlannerFSM
{
public:
    enum FSMExecState
    {
        INIT,
        WAIT_TARGET,
        GEN_NEW_TRAJ,
        REPLAN_TRAJ,
        EXEC_TRAJ,
        EMERGENCY_STOP,
        SEQUENTIAL_START
    };

    void init(ros::NodeHandle& nh, ros::NodeHandle& pnh);

private:
    void loadParameters(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    void odomCallback(const nav_msgs::OdometryConstPtr& msg);
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
    void goalCallback(const geometry_msgs::PoseStampedConstPtr& msg);
    void execFSMCallback(const ros::TimerEvent& event);
    void safetyCallback(const ros::TimerEvent& event);

    void changeFSMExecState(FSMExecState new_state, const std::string& caller);
    std::string stateName(FSMExecState state) const;
    void printFSMExecState() const;

    bool planCurrentGoal(bool new_traj);
    void publishPlannerOutputs();

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Subscriber odom_sub_;
    ros::Subscriber cloud_sub_;
    ros::Subscriber goal_sub_;
    ros::Subscriber extra_goal_sub_;
    ros::Publisher path_pub_;
    ros::Publisher searched_nodes_pub_;
    ros::Publisher debug_occupied_pub_;
    ros::Publisher debug_inflated_pub_;
    ros::Timer exec_timer_;
    ros::Timer safety_timer_;

    LocalPlannerManager manager_;
    std::shared_ptr<PathOptimizer> path_optimizer_;

    FSMExecState exec_state_{INIT};
    int continuously_called_times_{0};

    bool have_odom_{false};
    bool have_map_{false};
    bool have_target_{false};
    bool have_trigger_{true};
    bool have_recv_pre_agent_{true};
    bool have_new_target_{false};
    bool request_replan_{false};

    Eigen::Vector3d latest_goal_{0.0, 0.0, 0.0};

    std::string odom_topic_{"/fastnav/state/odom"};
    std::string cloud_topic_{"/fastnav/perception/cloud_filtered"};
    std::string goal_topic_{"/move_base_simple/goal"};
    std::string extra_goal_topic_{"/fastnav/planner/goal"};
    std::string path_topic_{"/fastnav/planner/path"};
    std::string searched_nodes_topic_{"/fastnav/planner/searched_nodes"};
    std::string debug_occupied_cloud_topic_{"/fastnav/planner/debug_occupied_cloud"};
    std::string debug_inflated_cloud_topic_{"/fastnav/planner/debug_inflated_cloud"};

    double exec_rate_{20.0};
    double safety_rate_{10.0};
    bool sequential_start_enable_{false};
    PathOptimizer::Config optimizer_config_;
};

}  // namespace fastnav_planner
