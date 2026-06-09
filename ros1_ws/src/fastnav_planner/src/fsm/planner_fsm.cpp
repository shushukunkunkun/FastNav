/*
 * planner_fsm.cpp
 *
 * 本文件实现 PlannerFSM 类的主要行为。
 * PlannerFSM 是 FastNav 局部规划模块的状态机入口，职责类似 EGO-Planner 中的 replanning_fsm：
 * 1. 通过 ROS topic 接收 odom、filtered cloud 和 goal；
 * 2. 维护规划状态 exec_state_，包括 INIT、WAIT_TARGET、GEN_NEW_TRAJ、REPLAN_TRAJ、EXEC_TRAJ、EMERGENCY_STOP；
 * 3. 持有 LocalPlannerManager::Ptr planner_manager_，把地图更新、A* 搜索、路径优化等算法工作交给 manager；
 * 4. 通过 timer 周期性推进状态机、发布 debug map，并在需要时触发重新规划；
 * 5. 发布 nav_msgs::Path、searched nodes、planner 内部 occupied / inflated debug cloud。
 *
 * 主要属性关系：
 * - odom_sub_ / cloud_sub_ / goal_sub_ 负责输入标准 FastNav 接口；
 * - path_pub_ / searched_nodes_pub_ / debug_*_pub_ 负责 RViz 和后续控制模块消费的输出；
 * - have_odom_、have_map_、have_target_、request_replan_ 等布尔量描述状态机前置条件；
 * - end_pt_ / local_target_pt_ 保存目标抽象，planner_manager_ 保存真正的地图、A*、优化器实例。
 */

#include "fastnav_planner/fsm/planner_fsm.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include <geometry_msgs/Point.h>
#include <traj_utils/minco/gcopter/geo_utils.hpp>

namespace fastnav_planner
{

// 初始化 PlannerFSM：创建 manager，读取参数，建立订阅 / 发布接口，并启动状态机定时器。
void PlannerFSM::init(ros::NodeHandle& nh, ros::NodeHandle& pnh)
{
    nh_ = nh;
    pnh_ = pnh;

    planner_manager_ = std::make_shared<LocalPlannerManager>();
    planner_manager_->init(nh_, pnh_);

    loadParameters(nh_, pnh_);

    exec_state_ = INIT;

    odom_sub_ = nh_.subscribe(odom_topic_, 20, &PlannerFSM::odometryCallback, this);
    cloud_sub_ = nh_.subscribe(cloud_topic_, 2, &PlannerFSM::cloudCallback, this);

    ros::NodeHandle goal_nh(nh_);
    goal_nh.setCallbackQueue(&goal_callback_queue_);
    goal_sub_ = goal_nh.subscribe(goal_topic_, 1, &PlannerFSM::waypointCallback, this);
    extra_goal_sub_ = goal_nh.subscribe(extra_goal_topic_, 1, &PlannerFSM::waypointCallback, this);
    goal_spinner_.reset(new ros::AsyncSpinner(1, &goal_callback_queue_));
    goal_spinner_->start();

    trigger_sub_ = nh_.subscribe(trigger_topic_, 1, &PlannerFSM::triggerCallback, this);

    path_pub_ = nh_.advertise<nav_msgs::Path>(path_topic_, 1, true);
    minco_traj_pub_ = nh_.advertise<traj_utils::MincoTrajectory>(minco_trajectory_topic_, 1, true);
    heartbeat_pub_ = nh_.advertise<std_msgs::Empty>(heartbeat_topic_, 10);

    if (debug_enable_)
    {
        searched_nodes_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(searched_nodes_topic_, 1, true);
        debug_occupied_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(debug_occupied_cloud_topic_, 1, true);
        debug_inflated_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(debug_inflated_cloud_topic_, 1, true);
        debug_corridor_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(debug_corridor_topic_, 1, true);
        fsm_state_pub_ = nh_.advertise<std_msgs::String>(fsm_state_topic_, 1, true);
        local_target_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(local_target_topic_, 1, true);
    }

    exec_timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(1.0, exec_rate_)),
                                  &PlannerFSM::execFSMCallback,
                                  this);
    safety_timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(1.0, safety_rate_)),
                                    &PlannerFSM::checkCollisionCallback,
                                    this);
    if (debug_enable_ && debug_map_publish_rate_ > 0.0)
    {
        debug_map_timer_ = nh_.createTimer(ros::Duration(1.0 / debug_map_publish_rate_),
                                           &PlannerFSM::debugMapCallback,
                                           this);
    }

    ROS_INFO("[FastNav][PlannerFSM] odom: %s", odom_topic_.c_str());
    ROS_INFO("[FastNav][PlannerFSM] cloud: %s", cloud_topic_.c_str());
    ROS_INFO("[FastNav][PlannerFSM] goal: %s and %s", goal_topic_.c_str(), extra_goal_topic_.c_str());
    ROS_INFO("[FastNav][PlannerFSM] trigger: %s", trigger_topic_.c_str());
    ROS_INFO("[FastNav][PlannerFSM] path output: %s", path_topic_.c_str());
    ROS_INFO("[FastNav][PlannerFSM] MINCO trajectory output: %s", minco_trajectory_topic_.c_str());
    ROS_INFO("[FastNav][PlannerFSM] heartbeat output: %s", heartbeat_topic_.c_str());
    ROS_INFO("[FastNav][PlannerFSM] debug output: %s", debug_enable_ ? "enabled" : "disabled");
    if (debug_enable_)
    {
        ROS_INFO("[FastNav][PlannerFSM] FSM state output: %s", fsm_state_topic_.c_str());
        ROS_INFO("[FastNav][PlannerFSM] local target output: %s", local_target_topic_.c_str());
        ROS_INFO("[FastNav][PlannerFSM] safe corridor output: %s", debug_corridor_topic_.c_str());
    }
}

// 读取状态机和 ROS topic 参数；全局参数来自 launch 加载的 yaml，私有参数用于局部覆盖。
void PlannerFSM::loadParameters(ros::NodeHandle& nh, ros::NodeHandle& pnh)
{
    nh.param<std::string>("/local_planner/odom_topic", odom_topic_, odom_topic_);
    nh.param<std::string>("/local_planner/cloud_topic", cloud_topic_, cloud_topic_);
    nh.param<std::string>("/local_planner/goal_topic", goal_topic_, goal_topic_);
    nh.param<std::string>("/local_planner/extra_goal_topic", extra_goal_topic_, extra_goal_topic_);
    nh.param<std::string>("/local_planner/trigger_topic", trigger_topic_, trigger_topic_);
    nh.param<std::string>("/local_planner/path_topic", path_topic_, path_topic_);
    nh.param<std::string>("/local_planner/searched_nodes_topic", searched_nodes_topic_, searched_nodes_topic_);
    nh.param<std::string>("/local_planner/debug_occupied_cloud_topic", debug_occupied_cloud_topic_, debug_occupied_cloud_topic_);
    nh.param<std::string>("/local_planner/debug_inflated_cloud_topic", debug_inflated_cloud_topic_, debug_inflated_cloud_topic_);
    nh.param<std::string>("/local_planner/debug_corridor_topic", debug_corridor_topic_, debug_corridor_topic_);
    nh.param<std::string>("/local_planner/minco_trajectory_topic", minco_trajectory_topic_, minco_trajectory_topic_);
    nh.param<std::string>("/local_planner/heartbeat_topic", heartbeat_topic_, heartbeat_topic_);
    nh.param<std::string>("/local_planner/fsm_state_topic", fsm_state_topic_, fsm_state_topic_);
    nh.param<std::string>("/local_planner/local_target_topic", local_target_topic_, local_target_topic_);
    nh.param<bool>("/local_planner/debug/enable", debug_enable_, debug_enable_);
    nh.param<double>("/local_planner/fsm/exec_rate", exec_rate_, exec_rate_);
    nh.param<double>("/local_planner/fsm/safety_rate", safety_rate_, safety_rate_);
    nh.param<double>("/local_planner/fsm/collision_check_step", collision_check_step_, collision_check_step_);
    nh.param<double>("/local_planner/fsm/collision_check_horizon_ratio", collision_check_horizon_ratio_, collision_check_horizon_ratio_);
    nh.param<double>("/local_planner/fsm/emergency_time", emergency_time_, emergency_time_);
    nh.param<double>("/local_planner/fsm/planning_horizon", planning_horizon_, planning_horizon_);
    nh.param<double>("/local_planner/fsm/goal_tolerance", goal_tolerance_, goal_tolerance_);
    nh.param<bool>("/local_planner/fsm/replan_time_auto", replan_time_auto_, replan_time_auto_);
    nh.param<double>("/local_planner/fsm/replan_time_ratio", replan_time_ratio_, replan_time_ratio_);
    nh.param<double>("/local_planner/fsm/replan_time", replan_time_, replan_time_);
    nh.param<int>("/local_planner/fsm/replan_trial_times", replan_trial_times_, replan_trial_times_);
    nh.param<double>("/local_planner/fsm/no_replan_distance", no_replan_distance_, no_replan_distance_);
    nh.param<double>("/local_planner/optimizer/feasibility/max_vel", max_vel_, max_vel_);
    nh.param<double>("/planner/planning_horizon", planning_horizon_, planning_horizon_);
    nh.param<double>("/planner/goal_tolerance", goal_tolerance_, goal_tolerance_);
    nh.param<double>("/planner/physical_limits/max_vel", max_vel_, max_vel_);
    nh.param<double>("/planner/physical_limits/max_acc", max_acc_, max_acc_);
    nh.param<double>("/local_planner/debug_map_publish_rate", debug_map_publish_rate_, debug_map_publish_rate_);
    nh.param<bool>("/local_planner/fsm/have_trigger", have_trigger_, have_trigger_);

    pnh.param<std::string>("odom_topic", odom_topic_, odom_topic_);
    pnh.param<std::string>("cloud_topic", cloud_topic_, cloud_topic_);
    pnh.param<std::string>("goal_topic", goal_topic_, goal_topic_);
    pnh.param<std::string>("extra_goal_topic", extra_goal_topic_, extra_goal_topic_);
    pnh.param<std::string>("trigger_topic", trigger_topic_, trigger_topic_);
    pnh.param<std::string>("minco_trajectory_topic", minco_trajectory_topic_, minco_trajectory_topic_);
    pnh.param<std::string>("heartbeat_topic", heartbeat_topic_, heartbeat_topic_);
    pnh.param<std::string>("fsm_state_topic", fsm_state_topic_, fsm_state_topic_);
    pnh.param<std::string>("local_target_topic", local_target_topic_, local_target_topic_);
    pnh.param<bool>("debug/enable", debug_enable_, debug_enable_);
    pnh.param<double>("fsm/exec_rate", exec_rate_, exec_rate_);
    pnh.param<double>("fsm/safety_rate", safety_rate_, safety_rate_);
    pnh.param<double>("fsm/collision_check_step", collision_check_step_, collision_check_step_);
    pnh.param<double>("fsm/collision_check_horizon_ratio", collision_check_horizon_ratio_, collision_check_horizon_ratio_);
    pnh.param<double>("fsm/emergency_time", emergency_time_, emergency_time_);
    pnh.param<double>("fsm/planning_horizon", planning_horizon_, planning_horizon_);
    pnh.param<double>("fsm/goal_tolerance", goal_tolerance_, goal_tolerance_);
    pnh.param<bool>("fsm/replan_time_auto", replan_time_auto_, replan_time_auto_);
    pnh.param<double>("fsm/replan_time_ratio", replan_time_ratio_, replan_time_ratio_);
    pnh.param<double>("fsm/replan_time", replan_time_, replan_time_);
    pnh.param<int>("fsm/replan_trial_times", replan_trial_times_, replan_trial_times_);
    pnh.param<double>("fsm/no_replan_distance", no_replan_distance_, no_replan_distance_);
    pnh.param<double>("max_vel", max_vel_, max_vel_);
    pnh.param<double>("max_acc", max_acc_, max_acc_);
    pnh.param<double>("debug_map_publish_rate", debug_map_publish_rate_, debug_map_publish_rate_);
    pnh.param<bool>("fsm/have_trigger", have_trigger_, have_trigger_);

    collision_check_horizon_ratio_ = std::min(1.0, std::max(0.05, collision_check_horizon_ratio_));
    emergency_time_ = std::max(0.0, emergency_time_);
    planning_horizon_ = std::max(0.1, planning_horizon_);
    goal_tolerance_ = std::max(0.05, goal_tolerance_);
    replan_time_ratio_ = std::min(1.0, std::max(0.05, replan_time_ratio_));
    max_vel_ = std::max(0.1, max_vel_);
    max_acc_ = std::max(0.1, max_acc_);
    if (replan_time_auto_)
    {
        replan_time_ = replan_time_ratio_ * planning_horizon_ / max_vel_;
    }
    replan_time_ = std::max(0.1, replan_time_);
    replan_trial_times_ = std::max(1, replan_trial_times_);
    no_replan_distance_ = std::max(0.05, no_replan_distance_);

    ROS_INFO("[FastNav][PlannerFSM] replan_time=%.3fs, trial_times=%d, auto=%d, ratio=%.3f, horizon=%.3fm, max_vel=%.3fm/s",
             replan_time_, replan_trial_times_, replan_time_auto_, replan_time_ratio_, planning_horizon_, max_vel_);
}

// 处理 FastNav 标准里程计输入，并把最新 odom 转交给 manager 作为地图中心和规划起点来源。
void PlannerFSM::odometryCallback(const nav_msgs::OdometryConstPtr& msg)
{
    const bool had_odom = have_odom_;
    const ros::Time current_stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    const Eigen::Vector3d current_vel(msg->twist.twist.linear.x,
                                      msg->twist.twist.linear.y,
                                      msg->twist.twist.linear.z);

    odom_pos_ = Eigen::Vector3d(msg->pose.pose.position.x,
                                msg->pose.pose.position.y,
                                msg->pose.pose.position.z);
    odom_vel_ = current_vel;
    odom_orient_ = Eigen::Quaterniond(msg->pose.pose.orientation.w,
                                      msg->pose.pose.orientation.x,
                                      msg->pose.pose.orientation.y,
                                      msg->pose.pose.orientation.z);
    if (odom_orient_.norm() > 1e-6)
    {
        odom_orient_.normalize();
    }
    else
    {
        odom_orient_ = Eigen::Quaterniond::Identity();
    }

    if (had_odom && !last_odom_stamp_.isZero())
    {
        const double dt = (current_stamp - last_odom_stamp_).toSec();
        if (dt > 1e-4)
        {
            odom_acc_ = (odom_vel_ - last_odom_vel_) / dt;
        }
    }
    else
    {
        odom_acc_.setZero();
    }

    last_odom_stamp_ = current_stamp;
    last_odom_vel_ = odom_vel_;

    have_odom_ = true;
    planner_manager_->updateOdom(msg);

    if (!have_target_)
    {
        init_pt_ = odom_pos_;
    }
}

// 处理滤波后的 odom 坐标系点云；manager 内部会更新 VoxelMap，并在执行中按需触发重规划。
void PlannerFSM::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    planner_manager_->updateCloud(msg);
    have_map_ = planner_manager_->hasMap();

    if (planner_manager_->replanOnCloud() && have_target_ && exec_state_ == EXEC_TRAJ)
    {
        request_replan_ = true;
    }
}

// 处理目标点输入；目标必须已经在 planner frame 中，当前版本不在 FSM 内做 TF 转换。
void PlannerFSM::waypointCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    if (!msg->header.frame_id.empty() && msg->header.frame_id != planner_manager_->frameId())
    {
        ROS_WARN("[FastNav][PlannerFSM] Goal frame is %s, expected %s. No TF conversion is done in planner.",
                 msg->header.frame_id.c_str(), planner_manager_->frameId().c_str());
        return;
    }

    const Eigen::Vector3d goal(msg->pose.position.x,
                               msg->pose.position.y,
                               msg->pose.position.z);

    {
        std::lock_guard<std::mutex> lock(pending_goal_mutex_);
        pending_goal_pt_ = goal;
        pending_goal_available_ = true;
    }
    goal_preempt_requested_.store(true);

    ROS_INFO("\033[1;34m[FastNav][PlannerFSM] Goal queued: [%.2f, %.2f, %.2f] in frame %s.\033[0m",
            goal.x(), goal.y(), goal.z(), planner_manager_->frameId().c_str());
}

bool PlannerFSM::applyPendingGoal(const std::string& caller)
{
    Eigen::Vector3d goal;
    {
        std::lock_guard<std::mutex> lock(pending_goal_mutex_);
        if (!pending_goal_available_)
        {
            return false;
        }

        goal = pending_goal_pt_;
        pending_goal_available_ = false;
    }

    goal_preempt_requested_.store(false);

    init_pt_ = have_odom_ ? odom_pos_ : init_pt_;
    end_pt_ = goal;
    end_vel_.setZero();
    local_target_pt_ = end_pt_;
    local_target_vel_ = end_vel_;
    wps_.clear();
    wps_.push_back(end_pt_);
    current_wp_ = 0;
    have_target_ = true;
    have_new_target_ = true;
    request_replan_ = true;
    have_published_local_target_ = false;

    ROS_INFO("\033[1;34m[FastNav][PlannerFSM] Goal accepted: [%.2f, %.2f, %.2f] in frame %s.\033[0m",
            end_pt_.x(), end_pt_.y(), end_pt_.z(), planner_manager_->frameId().c_str());

    if (have_odom_)
    {
        if (!planner_manager_->planGlobalTraj(odom_pos_,
                                              odom_vel_,
                                              odom_acc_,
                                              end_pt_,
                                              end_vel_,
                                              Eigen::Vector3d::Zero()))
        {
            ROS_WARN("[FastNav][PlannerFSM] Global reference generation failed: %s",
                     planner_manager_->lastError().c_str());
        }
    }

    // 新目标表示任务级目标已经改变，必须打断当前规划上下文并从当前 odom 重新生成全局参考和局部轨迹。
    if (exec_state_ != INIT)
    {
        changeFSMExecState(GEN_NEW_TRAJ, caller);
    }

    return true;
}

// 处理外部触发信号；仿真默认 have_trigger_=true，真实实验可将其设为 false 等待这个回调。
void PlannerFSM::triggerCallback(const geometry_msgs::PoseStampedConstPtr& /*msg*/)
{
    have_trigger_ = true;
    if (have_odom_)
    {
        init_pt_ = odom_pos_;
    }

    ROS_INFO("[FastNav][PlannerFSM] Trigger received.");
}

// 状态机主循环：根据 odom / map / target / replan 标志在各状态之间转换，并调用 EGO 风格规划入口。
void PlannerFSM::execFSMCallback(const ros::TimerEvent& /*event*/)
{
    publishHeartbeat();
    publishFSMState();

    if (exec_state_ != INIT && goal_preempt_requested_.load())
    {
        applyPendingGoal("GOAL");
    }

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
            changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        }
        break;

    case GEN_NEW_TRAJ:
        if (hasReachedGoal())
        {
            finishCurrentTarget("FSM");
            changeFSMExecState(WAIT_TARGET, "FSM");
            break;
        }
        if (planFromGlobalTraj(1))
        {
            if (!applyPendingGoal("GOAL_PREEMPT"))
            {
                changeFSMExecState(EXEC_TRAJ, "FSM");
            }
        }
        else
        {
            if (!applyPendingGoal("GOAL_PREEMPT"))
            {
                changeFSMExecState(GEN_NEW_TRAJ, "FSM");
            }
        }
        break;

    case REPLAN_TRAJ:
        if (hasReachedGoal())
        {
            finishCurrentTarget("FSM");
            changeFSMExecState(WAIT_TARGET, "FSM");
            break;
        }
        if (planFromCurrentTraj(replan_trial_times_))
        {
            if (!applyPendingGoal("GOAL_PREEMPT"))
            {
                changeFSMExecState(EXEC_TRAJ, "FSM");
            }
        }
        else
        {
            if (!applyPendingGoal("GOAL_PREEMPT"))
            {
                changeFSMExecState(REPLAN_TRAJ, "FSM");
            }
        }
        break;

    case EXEC_TRAJ:
        if (!have_target_)
        {
            changeFSMExecState(WAIT_TARGET, "FSM");
        }
        else if (hasReachedGoal())
        {
            finishCurrentTarget("FSM");
            changeFSMExecState(WAIT_TARGET, "FSM");
        }
        else if (have_new_target_ && request_replan_)
        {
            changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        }
        else
        {
            // EGO 风格主动重规划：
            // 1. 点云更新只改变地图，不直接触发 request_replan_；
            // 2. 当前局部轨迹执行超过 $T_r$ 后，若还没有接近最终目标，则滚动重规划；
            // 3. 若 local target 已经是最终目标，且当前位置距终点小于 $d_{no}$，则不再主动重规划，
            //    避免末端不断替换轨迹造成急刹和抖动。
            const auto& local_data = planner_manager_->local_data_;
            if (local_data.valid_ && !local_data.start_time_.isZero() && local_data.duration_ > 1.0e-6)
            {
                double t_cur = (ros::Time::now() - local_data.start_time_).toSec();
                t_cur = std::min(local_data.duration_, std::max(0.0, t_cur));
                const Eigen::Vector3d exec_pos = local_data.getPosition(t_cur);
                const bool local_target_is_goal = (local_target_pt_ - end_pt_).norm() < 1.0e-3;
                const bool far_from_goal = (end_pt_ - exec_pos).norm() > no_replan_distance_;

                if (local_target_is_goal)
                {
                    if (t_cur > local_data.duration_ - 1.0e-2)
                    {
                        finishCurrentTarget("FSM");
                        changeFSMExecState(WAIT_TARGET, "FSM");
                    }
                    else if (far_from_goal && t_cur > replan_time_)
                    {
                        changeFSMExecState(REPLAN_TRAJ, "FSM");
                    }
                }
                else if (t_cur > replan_time_)
                {
                    changeFSMExecState(REPLAN_TRAJ, "FSM");
                }
            }
        }
        break;

    case EMERGENCY_STOP:
        if (have_odom_)
        {
            callEmergencyStop(planner_manager_->currentPosition());
        }
        if (have_odom_ && have_map_ && have_target_)
        {
            changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        }
        break;
    }
}

// 安全检查定时器入口；当前 planner 只生成路径，后续可在这里检测当前路径是否被新障碍截断。
void PlannerFSM::checkCollisionCallback(const ros::TimerEvent& /*event*/)
{
    // 与 EGO-Planner 的 checkCollisionCallback 对齐：这里不做底层控制，只判断当前路径是否仍然安全。
    // 若当前 path 被新地图截断，先尝试从当前 odom 重新规划；失败时切入 EMERGENCY_STOP。
    if (exec_state_ != EXEC_TRAJ || !planner_manager_->hasPath() || !planner_manager_->hasMap())
    {
        return;
    }

    double collision_time_from_now = 0.0;
    if (planner_manager_->isCurrentTrajectoryCollisionFree(collision_check_step_,
                                                           collision_check_horizon_ratio_,
                                                           touch_goal_,
                                                           collision_time_from_now))
    {
        return;
    }

    ROS_WARN("[FastNav][PlannerFSM] Current trajectory is in collision after %.2fs. Try replan.",
             collision_time_from_now);
    if (planFromCurrentTraj(replan_trial_times_))
    {
        changeFSMExecState(EXEC_TRAJ, "SAFETY");
        return;
    }

    if (collision_time_from_now <= emergency_time_)
    {
        ROS_WARN("[FastNav][PlannerFSM] Replan failed and collision is within emergency_time=%.2fs.",
                 emergency_time_);
        changeFSMExecState(EMERGENCY_STOP, "SAFETY");
    }
    else
    {
        changeFSMExecState(REPLAN_TRAJ, "SAFETY");
    }
}

// 按固定频率发布 planner 内部调试地图，避免每帧点云回调都刷 RViz 导致闪烁。
void PlannerFSM::debugMapCallback(const ros::TimerEvent& /*event*/)
{
    // 与 EGO-Planner 的 GridMap::visCallback 类似，debug 地图使用独立定时器发布。
    // cloud callback 只更新内部 VoxelMap，RViz 固定频率收到局部地图快照，减少闪烁和突发发布。
    publishPlannerOutputs();
}

// 切换状态机状态，并记录连续进入同一状态的次数，便于后续做超时或失败统计。
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

// 将状态枚举转换为可读字符串，主要用于日志输出。
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
    }
    return "UNKNOWN";
}

// 周期性打印状态机当前状态和前置条件，方便定位“有目标但没规划”的原因。
void PlannerFSM::printFSMExecState() const
{
    ROS_INFO("[FastNav][PlannerFSM] state=%s, odom=%d, map=%d, target=%d",
             stateName(exec_state_).c_str(), have_odom_, have_map_, have_target_);
}

// 从全局目标生成新路径；当前第一版没有全局参考轨迹，因此直接规划到 end_pt_。
bool PlannerFSM::planFromGlobalTraj(int trial_times)
{
    if (have_odom_)
    {
        start_pt_ = odom_pos_;
        start_vel_ = odom_vel_;
        start_acc_ = odom_acc_;
        const double siny_cosp = 2.0 * (odom_orient_.w() * odom_orient_.z() + odom_orient_.x() * odom_orient_.y());
        const double cosy_cosp = 1.0 - 2.0 * (odom_orient_.y() * odom_orient_.y() + odom_orient_.z() * odom_orient_.z());
        start_yaw_ = Eigen::Vector3d(std::atan2(siny_cosp, cosy_cosp), 0.0, 0.0);
    }

    if (!planner_manager_->global_data_.valid_)
    {
        planner_manager_->planGlobalTraj(start_pt_,
                                         start_vel_,
                                         start_acc_,
                                         end_pt_,
                                         end_vel_,
                                         Eigen::Vector3d::Zero());
    }

    for (int i = 0; i < std::max(1, trial_times); ++i)
    {
        const bool use_random_init = i > 0 || continuously_called_times_ > 1;
        if (callReplan(false, use_random_init, i))
        {
            return true;
        }
    }

    return false;
}

// 从当前执行轨迹重新规划。与 EGO-Planner 一致，优先从当前已经接受的局部 MINCO 轨迹上取
// $p(t_c),v(t_c),a(t_c)$ 作为新轨迹起点状态，而不是直接使用最新 odom。
// 这样新轨迹和控制器正在跟踪的旧轨迹在切换时具有更好的位置、速度、加速度连续性。
bool PlannerFSM::planFromCurrentTraj(int trial_times)
{
    const auto& local_data = planner_manager_->local_data_;
    const ros::Time time_now = ros::Time::now();
    if (local_data.valid_ && !local_data.start_time_.isZero() && local_data.duration_ > 1.0e-6)
    {
        const double t_cur = std::min(local_data.duration_,
                                      std::max(0.0, (time_now - local_data.start_time_).toSec()));
        start_pt_ = local_data.getPosition(t_cur);
        start_vel_ = local_data.getVelocity(t_cur);
        start_acc_ = local_data.getAcceleration(t_cur);
    }
    else if (have_odom_)
    {
        start_pt_ = odom_pos_;
        start_vel_ = odom_vel_;
        start_acc_ = odom_acc_;
    }

    if (have_odom_)
    {
        const double siny_cosp = 2.0 * (odom_orient_.w() * odom_orient_.z() + odom_orient_.x() * odom_orient_.y());
        const double cosy_cosp = 1.0 - 2.0 * (odom_orient_.y() * odom_orient_.y() + odom_orient_.z() * odom_orient_.z());
        start_yaw_ = Eigen::Vector3d(std::atan2(siny_cosp, cosy_cosp), 0.0, 0.0);
    }

    // EGO-v2 风格三段式 fallback：
    // 1. 优先延续旧局部 MINCO 轨迹剩余段，并拼接 A* 到新的 local target，使新旧轨迹尽量连续；
    // 2. 若失败，放弃旧轨迹几何前缀，直接用整段 A* 路径生成 corridor/MINCO，摆脱旧轨迹形状限制；
    // 3. 若仍失败，在整段 A* 参考路径上做随机扰动，生成不同 corridor 拓扑和 MINCO 初值。
    if (callReplan(true, false, 0))
    {
        return true;
    }

    if (callReplan(false, false, 0))
    {
        return true;
    }

    for (int i = 0; i < std::max(1, trial_times); ++i)
    {
        if (callReplan(false, true, i + 1))
        {
            return true;
        }
    }

    return false;
}

// 统一规划入口：检查前置条件，选择局部目标，调用 manager 的 A* + PathOptimizer，并发布结果。
bool PlannerFSM::callReplan(bool use_current_path, bool use_random_init, int retry_index)
{
    if (!have_odom_ || !have_map_ || !have_target_)
    {
        ROS_WARN_THROTTLE(1.0,
                          "[FastNav][PlannerFSM] Cannot plan yet. odom=%d, map=%d, target=%d",
                          have_odom_, have_map_, have_target_);
        return false;
    }

    getLocalTarget();
    if (goal_preempt_requested_.load())
    {
        return false;
    }

    LocalPlannerManager::ReplanOptions options;
    options.use_current_traj = use_current_path;
    options.use_random_init = use_random_init;
    options.has_start_state = true;
    options.start_pos = start_pt_;
    options.start_vel = start_vel_;
    options.start_acc = start_acc_;
    options.goal_vel = local_target_vel_;
    options.goal_acc.setZero();
    options.touch_goal = touch_goal_;
    options.attempt = continuously_called_times_ + std::max(0, retry_index);
    options.continuous_failures = continuously_called_times_;
    options.preempt_requested = [this]() {
        return goal_preempt_requested_.load();
    };

    const bool success = planner_manager_->planToGoal(local_target_pt_, options);
    if (goal_preempt_requested_.load())
    {
        ROS_INFO("[FastNav][PlannerFSM] Discard old planning result because a new goal is pending.");
        return false;
    }

    if (debug_enable_ && searched_nodes_pub_)
    {
        searched_nodes_pub_.publish(planner_manager_->getSearchedNodesCloud());
    }

    if (!success)
    {
        ROS_WARN_THROTTLE(1.0, "[FastNav][PlannerFSM] Planning failed: %s", planner_manager_->lastError().c_str());
        return false;
    }

    publishTrajectory();
    request_replan_ = false;
    have_new_target_ = false;

    ROS_INFO("[FastNav][PlannerFSM] Path planned and published.");
    return true;
}

// 当前 path 表达下，急停等价于发布一个停在当前位置的单点路径；后续可替换成带速度约束的轨迹。
bool PlannerFSM::callEmergencyStop(const Eigen::Vector3d& stop_pos)
{
    planner_manager_->setPath(std::vector<Eigen::Vector3d>{stop_pos});
    publishTrajectory();
    ROS_WARN_THROTTLE(1.0,
                      "[FastNav][PlannerFSM] Emergency stop path published at [%.2f, %.2f, %.2f].",
                      stop_pos.x(), stop_pos.y(), stop_pos.z());
    return true;
}

// 沿任务参考线选择局部目标。
// EGO-Planner 会沿 global_data_ 的全局轨迹向前扫描，寻找满足 $\|p_g(t)-p_s\|\ge H$ 的点；
// FastNav 这里读取 manager 维护的轻量全局 MINCO 参考轨迹 $p_g(t)$。
void PlannerFSM::getLocalTarget()
{
    const auto& global_data = planner_manager_->global_data_;
    if (global_data.valid_ && global_data.duration_ > 1.0e-6)
    {
        const double t_step = std::max(0.02, planning_horizon_ / 20.0 / max_vel_);
        double dist_min = std::numeric_limits<double>::infinity();
        double dist_min_t = global_data.last_progress_time_;
        double t = global_data.last_progress_time_;

        for (; t < global_data.duration_; t += t_step)
        {
            Eigen::Vector3d pos_t = global_data.getPosition(t);
            double dist = (pos_t - start_pt_).norm();

            if (t < global_data.last_progress_time_ + 1.0e-5 && dist > planning_horizon_)
            {
                // 与 EGO 的 corner case 一致：如果 last_progress_time_ 对应点已经在 horizon 外，
                // 就继续向前找一个重新进入 horizon 的点，避免局部目标跳到身后或远处。
                for (; t < global_data.duration_; t += t_step)
                {
                    const Eigen::Vector3d pos_temp = global_data.getPosition(t);
                    const double dist_temp = (pos_temp - start_pt_).norm();
                    if (dist_temp < planning_horizon_)
                    {
                        pos_t = pos_temp;
                        dist = dist_temp;
                        break;
                    }
                }
            }

            if (dist < dist_min)
            {
                dist_min = dist;
                dist_min_t = t;
            }

            if (dist >= planning_horizon_)
            {
                local_target_pt_ = pos_t;
                planner_manager_->global_data_.last_progress_time_ = dist_min_t;
                touch_goal_ = false;
                break;
            }
        }

        if (t >= global_data.duration_)
        {
            local_target_pt_ = end_pt_;
            planner_manager_->global_data_.last_progress_time_ = global_data.duration_;
            touch_goal_ = true;
        }

        const double brake_dist = max_vel_ * max_vel_ / (2.0 * max_acc_);
        if ((end_pt_ - local_target_pt_).norm() < brake_dist || touch_goal_)
        {
            local_target_vel_.setZero();
        }
        else
        {
            local_target_vel_ = global_data.getVelocity(std::min(t, global_data.duration_));
            if (local_target_vel_.norm() < 1.0e-3)
            {
                const Eigen::Vector3d dir = (end_pt_ - start_pt_).normalized();
                local_target_vel_ = dir * max_vel_;
            }
        }
        publishLocalTargetIfChanged();
        return;
    }

    // 没有全局参考轨迹时回退到直线 horizon 切片，保证系统仍能工作。
    const Eigen::Vector3d to_goal = end_pt_ - start_pt_;
    const double dist_to_goal = to_goal.norm();

    if (dist_to_goal <= planning_horizon_ + 1.0e-3)
    {
        local_target_pt_ = end_pt_;
        local_target_vel_ = end_vel_;
        touch_goal_ = true;
        publishLocalTargetIfChanged();
        return;
    }

    const Eigen::Vector3d dir = to_goal / std::max(dist_to_goal, 1.0e-6);
    local_target_pt_ = start_pt_ + dir * planning_horizon_;
    touch_goal_ = false;

    // 若离最终目标已经进入刹车距离 $d_b=v_{max}^2/(2a_{max})$，局部目标速度设为 0；
    // 否则给中间 local target 一个沿全局参考线方向的巡航速度，避免 MINCO 每段都在 horizon 处停车。
    const double brake_dist = max_vel_ * max_vel_ / (2.0 * max_acc_);
    if ((end_pt_ - local_target_pt_).norm() < brake_dist)
    {
        local_target_vel_.setZero();
    }
    else
    {
        local_target_vel_ = dir * max_vel_;
    }
    publishLocalTargetIfChanged();
}

bool PlannerFSM::hasReachedGoal() const
{
    if (!have_target_ || !have_odom_)
    {
        return false;
    }

    return (odom_pos_ - end_pt_).norm() <= goal_tolerance_;
}

void PlannerFSM::finishCurrentTarget(const std::string& caller)
{
    ROS_INFO("[FastNav][PlannerFSM][%s] Target reached. dist=%.3f, tolerance=%.3f",
             caller.c_str(), (odom_pos_ - end_pt_).norm(), goal_tolerance_);

    have_target_ = false;
    have_new_target_ = false;
    request_replan_ = false;
    touch_goal_ = true;
    local_target_pt_ = end_pt_;
    local_target_vel_.setZero();
    end_vel_.setZero();
    have_published_local_target_ = false;
    planner_manager_->global_data_.reset();
}

// 发布当前路径和 A* 搜索节点。路径消息仍保持 nav_msgs::Path，内容来自 MINCO 采样点或几何回退路径。
void PlannerFSM::publishTrajectory()
{
    path_pub_.publish(planner_manager_->getPathMsg());
    if (debug_enable_ && searched_nodes_pub_)
    {
        searched_nodes_pub_.publish(planner_manager_->getSearchedNodesCloud());
    }
    publishMincoTrajectory();
}

// 发布 planner 内部维护的 occupied / inflated debug cloud，用于和独立 mapping 节点输出做对照。
void PlannerFSM::publishPlannerOutputs()
{
    if (!debug_enable_ || !planner_manager_->hasMap() || !debug_occupied_pub_ || !debug_inflated_pub_)
    {
        return;
    }

    debug_occupied_pub_.publish(planner_manager_->getDebugOccupiedCloud());
    debug_inflated_pub_.publish(planner_manager_->getDebugInflatedCloud());
    publishCorridorMarkers();
}

void PlannerFSM::publishCorridorMarkers()
{
    if (!debug_enable_ || !debug_corridor_pub_)
    {
        return;
    }

    visualization_msgs::MarkerArray markers;

    visualization_msgs::Marker delete_all;
    delete_all.header.stamp = ros::Time::now();
    delete_all.header.frame_id = "odom";
    delete_all.ns = "fastnav_safe_corridor";
    delete_all.id = 0;
    delete_all.action = visualization_msgs::Marker::DELETEALL;
    markers.markers.push_back(delete_all);

    const auto& local_data = planner_manager_->local_data_;
    if (!local_data.valid_ || local_data.corridor_.empty())
    {
        debug_corridor_pub_.publish(markers);
        return;
    }

    const ros::Time stamp = ros::Time::now();
    const std::string frame_id = local_data.frame_id_.empty() ? "odom" : local_data.frame_id_;
    constexpr double active_eps = 1.0e-5;

    int marker_id = 1;
    for (size_t corridor_id = 0; corridor_id < local_data.corridor_.size(); ++corridor_id)
    {
        const Eigen::MatrixX4d& hpoly = local_data.corridor_[corridor_id];
        Eigen::Matrix3Xd vertices;
        if (hpoly.rows() < 4 || hpoly.cols() != 4 || !geo_utils::enumerateVs(hpoly, vertices) || vertices.cols() < 2)
        {
            continue;
        }

        visualization_msgs::Marker line_marker;
        line_marker.header.stamp = stamp;
        line_marker.header.frame_id = frame_id;
        line_marker.ns = "fastnav_safe_corridor";
        line_marker.id = marker_id++;
        line_marker.type = visualization_msgs::Marker::LINE_LIST;
        line_marker.action = visualization_msgs::Marker::ADD;
        line_marker.pose.orientation.w = 1.0;
        line_marker.scale.x = 0.035;
        line_marker.color.r = 0.1f;
        line_marker.color.g = 0.75f;
        line_marker.color.b = 1.0f;
        line_marker.color.a = 0.85f;

        // 两个顶点若共同落在至少两个活跃平面上，则它们之间是 3D 凸多面体的一条边。
        // 这样可从 H-polytope 顶点集合恢复线框，RViz 中能看到 corridor 的真实几何范围。
        for (int i = 0; i < vertices.cols(); ++i)
        {
            const Eigen::Vector4d vi(vertices(0, i), vertices(1, i), vertices(2, i), 1.0);
            for (int j = i + 1; j < vertices.cols(); ++j)
            {
                const Eigen::Vector4d vj(vertices(0, j), vertices(1, j), vertices(2, j), 1.0);
                int shared_active_planes = 0;
                for (int row = 0; row < hpoly.rows(); ++row)
                {
                    const double di = std::abs(hpoly.row(row).dot(vi));
                    const double dj = std::abs(hpoly.row(row).dot(vj));
                    if (di < active_eps && dj < active_eps)
                    {
                        ++shared_active_planes;
                    }
                }

                if (shared_active_planes >= 2)
                {
                    geometry_msgs::Point p0;
                    p0.x = vi.x();
                    p0.y = vi.y();
                    p0.z = vi.z();
                    geometry_msgs::Point p1;
                    p1.x = vj.x();
                    p1.y = vj.y();
                    p1.z = vj.z();
                    line_marker.points.push_back(p0);
                    line_marker.points.push_back(p1);
                }
            }
        }

        if (!line_marker.points.empty())
        {
            markers.markers.push_back(line_marker);
        }
    }

    debug_corridor_pub_.publish(markers);
}

void PlannerFSM::publishMincoTrajectory()
{
    if (!minco_traj_pub_ || !planner_manager_->local_data_.valid_)
    {
        return;
    }

    const auto& local_data = planner_manager_->local_data_;
    const auto& traj = local_data.position_traj_.trajectory();
    constexpr int kOrder = 5;
    constexpr int kCoeffNum = kOrder + 1;
    const int piece_num = traj.getPieceNum();
    if (piece_num <= 0)
    {
        return;
    }

    traj_utils::MincoTrajectory msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = local_data.frame_id_;
    msg.traj_id = static_cast<uint32_t>(std::max(0, local_data.traj_id_));
    msg.order = kOrder;
    msg.start_time = local_data.start_time_;
    msg.duration.reserve(piece_num);
    msg.coef_x.reserve(piece_num * kCoeffNum);
    msg.coef_y.reserve(piece_num * kCoeffNum);
    msg.coef_z.reserve(piece_num * kCoeffNum);

    for (int i = 0; i < piece_num; ++i)
    {
        const auto& piece = traj[i];
        const auto& coeff = piece.getCoeffMat();
        msg.duration.push_back(piece.getDuration());
        for (int k = 0; k < kCoeffNum; ++k)
        {
            msg.coef_x.push_back(coeff(0, k));
            msg.coef_y.push_back(coeff(1, k));
            msg.coef_z.push_back(coeff(2, k));
        }
    }

    minco_traj_pub_.publish(msg);
}

void PlannerFSM::publishLocalTargetIfChanged()
{
    if (!debug_enable_ || !local_target_pub_)
    {
        return;
    }

    if (have_published_local_target_ &&
        (local_target_pt_ - last_published_local_target_pt_).norm() < 1.0e-3)
    {
        return;
    }

    geometry_msgs::PoseStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = planner_manager_->frameId();
    msg.pose.position.x = local_target_pt_.x();
    msg.pose.position.y = local_target_pt_.y();
    msg.pose.position.z = local_target_pt_.z();
    msg.pose.orientation.w = 1.0;
    local_target_pub_.publish(msg);

    last_published_local_target_pt_ = local_target_pt_;
    have_published_local_target_ = true;

    ROS_INFO("[FastNav][PlannerFSM] Local target updated: [%.2f, %.2f, %.2f], touch_goal=%d",
             local_target_pt_.x(), local_target_pt_.y(), local_target_pt_.z(), touch_goal_);
}

void PlannerFSM::publishHeartbeat()
{
    if (!heartbeat_pub_)
    {
        return;
    }

    std_msgs::Empty heartbeat;
    heartbeat_pub_.publish(heartbeat);
}

void PlannerFSM::publishFSMState()
{
    if (!debug_enable_ || !fsm_state_pub_)
    {
        return;
    }

    std_msgs::String msg;
    msg.data = stateName(exec_state_);
    fsm_state_pub_.publish(msg);
}

}  // namespace fastnav_planner
