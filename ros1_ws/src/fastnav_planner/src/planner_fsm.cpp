#include "fastnav_planner/planner_fsm.h"

#include <algorithm>

namespace fastnav_planner
{

void PlannerFSM::init(ros::NodeHandle& nh, ros::NodeHandle& pnh)
{
    nh_ = nh;
    pnh_ = pnh;

    manager_.init(nh_, pnh_);
    path_optimizer_ = std::make_shared<PathOptimizer>();

    loadParameters(nh_, pnh_);
    path_optimizer_->setConfig(optimizer_config_);

    exec_state_ = INIT;

    odom_sub_ = nh_.subscribe(odom_topic_, 20, &PlannerFSM::odomCallback, this);
    cloud_sub_ = nh_.subscribe(cloud_topic_, 2, &PlannerFSM::cloudCallback, this);
    goal_sub_ = nh_.subscribe(goal_topic_, 1, &PlannerFSM::goalCallback, this);
    extra_goal_sub_ = nh_.subscribe(extra_goal_topic_, 1, &PlannerFSM::goalCallback, this);

    path_pub_ = nh_.advertise<nav_msgs::Path>(path_topic_, 1, true);
    searched_nodes_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(searched_nodes_topic_, 1, true);
    debug_occupied_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(debug_occupied_cloud_topic_, 1, true);
    debug_inflated_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(debug_inflated_cloud_topic_, 1, true);

    exec_timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(1.0, exec_rate_)),
                                  &PlannerFSM::execFSMCallback,
                                  this);
    safety_timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(1.0, safety_rate_)),
                                    &PlannerFSM::safetyCallback,
                                    this);

    ROS_INFO("[FastNav][PlannerFSM] odom: %s", odom_topic_.c_str());
    ROS_INFO("[FastNav][PlannerFSM] cloud: %s", cloud_topic_.c_str());
    ROS_INFO("[FastNav][PlannerFSM] goal: %s and %s", goal_topic_.c_str(), extra_goal_topic_.c_str());
    ROS_INFO("[FastNav][PlannerFSM] path output: %s", path_topic_.c_str());
}

void PlannerFSM::loadParameters(ros::NodeHandle& nh, ros::NodeHandle& pnh)
{
    nh.param<std::string>("/local_planner/odom_topic", odom_topic_, odom_topic_);
    nh.param<std::string>("/local_planner/cloud_topic", cloud_topic_, cloud_topic_);
    nh.param<std::string>("/local_planner/goal_topic", goal_topic_, goal_topic_);
    nh.param<std::string>("/local_planner/extra_goal_topic", extra_goal_topic_, extra_goal_topic_);
    nh.param<std::string>("/local_planner/path_topic", path_topic_, path_topic_);
    nh.param<std::string>("/local_planner/searched_nodes_topic", searched_nodes_topic_, searched_nodes_topic_);
    nh.param<std::string>("/local_planner/debug_occupied_cloud_topic", debug_occupied_cloud_topic_, debug_occupied_cloud_topic_);
    nh.param<std::string>("/local_planner/debug_inflated_cloud_topic", debug_inflated_cloud_topic_, debug_inflated_cloud_topic_);
    nh.param<double>("/local_planner/fsm/exec_rate", exec_rate_, exec_rate_);
    nh.param<double>("/local_planner/fsm/safety_rate", safety_rate_, safety_rate_);
    nh.param<bool>("/local_planner/fsm/have_trigger", have_trigger_, have_trigger_);
    nh.param<bool>("/local_planner/fsm/sequential_start_enable", sequential_start_enable_, sequential_start_enable_);
    nh.param<bool>("/local_planner/optimizer/enable", optimizer_config_.enable, optimizer_config_.enable);
    nh.param<bool>("/local_planner/optimizer/shortcut", optimizer_config_.shortcut, optimizer_config_.shortcut);
    nh.param<double>("/local_planner/optimizer/line_check_step", optimizer_config_.line_check_step, optimizer_config_.line_check_step);

    pnh.param<std::string>("odom_topic", odom_topic_, odom_topic_);
    pnh.param<std::string>("cloud_topic", cloud_topic_, cloud_topic_);
    pnh.param<std::string>("goal_topic", goal_topic_, goal_topic_);
    pnh.param<std::string>("extra_goal_topic", extra_goal_topic_, extra_goal_topic_);
    pnh.param<double>("fsm/exec_rate", exec_rate_, exec_rate_);
    pnh.param<double>("fsm/safety_rate", safety_rate_, safety_rate_);
    pnh.param<bool>("fsm/have_trigger", have_trigger_, have_trigger_);
    pnh.param<bool>("fsm/sequential_start_enable", sequential_start_enable_, sequential_start_enable_);
    pnh.param<bool>("optimizer/enable", optimizer_config_.enable, optimizer_config_.enable);
    pnh.param<bool>("optimizer/shortcut", optimizer_config_.shortcut, optimizer_config_.shortcut);
    pnh.param<double>("optimizer/line_check_step", optimizer_config_.line_check_step, optimizer_config_.line_check_step);
}

void PlannerFSM::odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
    have_odom_ = true;
    manager_.updateOdom(msg);
}

void PlannerFSM::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    manager_.updateCloud(msg);
    have_map_ = manager_.hasMap();
    publishPlannerOutputs();

    if (manager_.replanOnCloud() && have_target_ && exec_state_ == EXEC_TRAJ)
    {
        request_replan_ = true;
    }
}

void PlannerFSM::goalCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    if (!msg->header.frame_id.empty() && msg->header.frame_id != manager_.frameId())
    {
        ROS_WARN("[FastNav][PlannerFSM] Goal frame is %s, expected %s. No TF conversion is done in planner.",
                 msg->header.frame_id.c_str(), manager_.frameId().c_str());
        return;
    }

    latest_goal_ = Eigen::Vector3d(msg->pose.position.x,
                                   msg->pose.position.y,
                                   msg->pose.position.z);
    have_target_ = true;
    have_new_target_ = true;
    request_replan_ = true;

    ROS_INFO("[FastNav][PlannerFSM] Goal received: [%.2f, %.2f, %.2f] in frame %s.",
             latest_goal_.x(), latest_goal_.y(), latest_goal_.z(), manager_.frameId().c_str());

    if (exec_state_ == WAIT_TARGET)
    {
        changeFSMExecState(GEN_NEW_TRAJ, "GOAL");
    }
    else if (exec_state_ == EXEC_TRAJ)
    {
        changeFSMExecState(REPLAN_TRAJ, "GOAL");
    }
}

void PlannerFSM::execFSMCallback(const ros::TimerEvent& /*event*/)
{
    static int print_count = 0;
    if (++print_count >= static_cast<int>(std::max(1.0, exec_rate_)))
    {
        printFSMExecState();
        print_count = 0;
    }

    switch (exec_state_)
    {
    case INIT:
        if (have_odom_)
        {
            changeFSMExecState(WAIT_TARGET, "FSM");
        }
        break;

    case WAIT_TARGET:
        if (have_odom_ && have_map_ && have_target_ && have_trigger_)
        {
            changeFSMExecState(sequential_start_enable_ ? SEQUENTIAL_START : GEN_NEW_TRAJ, "FSM");
        }
        break;

    case SEQUENTIAL_START:
        if (have_recv_pre_agent_ && planCurrentGoal(true))
        {
            changeFSMExecState(EXEC_TRAJ, "FSM");
        }
        break;

    case GEN_NEW_TRAJ:
        if (planCurrentGoal(true))
        {
            changeFSMExecState(EXEC_TRAJ, "FSM");
        }
        else
        {
            changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        }
        break;

    case REPLAN_TRAJ:
        if (planCurrentGoal(false))
        {
            changeFSMExecState(EXEC_TRAJ, "FSM");
        }
        else
        {
            changeFSMExecState(REPLAN_TRAJ, "FSM");
        }
        break;

    case EXEC_TRAJ:
        if (!have_target_)
        {
            changeFSMExecState(WAIT_TARGET, "FSM");
        }
        else if (request_replan_)
        {
            changeFSMExecState(have_new_target_ ? GEN_NEW_TRAJ : REPLAN_TRAJ, "FSM");
        }
        break;

    case EMERGENCY_STOP:
        if (have_odom_ && have_map_ && have_target_)
        {
            changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        }
        break;
    }
}

void PlannerFSM::safetyCallback(const ros::TimerEvent& /*event*/)
{
    // 当前阶段 planner 只输出路径，不直接控制无人机；安全停止由 control 层最终执行。
    // 这里先保留与 EGO-Planner 相同的 safety timer 入口，后续可加入当前 path 碰撞预测并切入 EMERGENCY_STOP。
}

void PlannerFSM::changeFSMExecState(FSMExecState new_state, const std::string& caller)
{
    if (new_state == exec_state_)
    {
        ++continuously_called_times_;
    }
    else
    {
        continuously_called_times_ = 1;
    }

    const FSMExecState old_state = exec_state_;
    exec_state_ = new_state;
    ROS_INFO("[FastNav][PlannerFSM][%s] %s -> %s",
             caller.c_str(), stateName(old_state).c_str(), stateName(exec_state_).c_str());
}

std::string PlannerFSM::stateName(FSMExecState state) const
{
    switch (state)
    {
    case INIT:
        return "INIT";
    case WAIT_TARGET:
        return "WAIT_TARGET";
    case GEN_NEW_TRAJ:
        return "GEN_NEW_TRAJ";
    case REPLAN_TRAJ:
        return "REPLAN_TRAJ";
    case EXEC_TRAJ:
        return "EXEC_TRAJ";
    case EMERGENCY_STOP:
        return "EMERGENCY_STOP";
    case SEQUENTIAL_START:
        return "SEQUENTIAL_START";
    }
    return "UNKNOWN";
}

void PlannerFSM::printFSMExecState() const
{
    ROS_INFO("[FastNav][PlannerFSM] state=%s, odom=%d, map=%d, target=%d",
             stateName(exec_state_).c_str(), have_odom_, have_map_, have_target_);
}

bool PlannerFSM::planCurrentGoal(bool /*new_traj*/)
{
    if (!have_odom_ || !have_map_ || !have_target_)
    {
        ROS_WARN_THROTTLE(1.0,
                          "[FastNav][PlannerFSM] Cannot plan yet. odom=%d, map=%d, target=%d",
                          have_odom_, have_map_, have_target_);
        return false;
    }

    const bool success = manager_.planToGoal(latest_goal_);
    searched_nodes_pub_.publish(manager_.getSearchedNodesCloud());
    publishPlannerOutputs();

    if (!success)
    {
        ROS_WARN_THROTTLE(1.0, "[FastNav][PlannerFSM] Planning failed: %s", manager_.lastError().c_str());
        return false;
    }

    std::vector<Eigen::Vector3d> optimized_path;
    if (!path_optimizer_->optimize(manager_.getPath(), manager_.voxelMap(), optimized_path))
    {
        ROS_WARN_THROTTLE(1.0,
                          "[FastNav][PlannerFSM] Path optimization failed: %s. Use raw A* path.",
                          path_optimizer_->lastError().c_str());
    }
    else
    {
        manager_.setPath(optimized_path);
    }

    path_pub_.publish(manager_.getPathMsg());
    publishPlannerOutputs();
    request_replan_ = false;
    have_new_target_ = false;

    ROS_INFO("[FastNav][PlannerFSM] Path planned and published.");
    return true;
}

void PlannerFSM::publishPlannerOutputs()
{
    if (!manager_.hasMap())
    {
        return;
    }

    debug_occupied_pub_.publish(manager_.getDebugOccupiedCloud());
    debug_inflated_pub_.publish(manager_.getDebugInflatedCloud());
}

}  // namespace fastnav_planner
