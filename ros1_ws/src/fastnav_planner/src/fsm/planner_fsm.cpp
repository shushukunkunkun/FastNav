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
    goal_sub_ = nh_.subscribe(goal_topic_, 1, &PlannerFSM::waypointCallback, this);
    extra_goal_sub_ = nh_.subscribe(extra_goal_topic_, 1, &PlannerFSM::waypointCallback, this);
    trigger_sub_ = nh_.subscribe(trigger_topic_, 1, &PlannerFSM::triggerCallback, this);

    path_pub_ = nh_.advertise<nav_msgs::Path>(path_topic_, 1, true);
    searched_nodes_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(searched_nodes_topic_, 1, true);
    debug_occupied_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(debug_occupied_cloud_topic_, 1, true);
    debug_inflated_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(debug_inflated_cloud_topic_, 1, true);
    minco_traj_pub_ = nh_.advertise<traj_utils::MincoTrajectory>(minco_trajectory_topic_, 1, true);
    heartbeat_pub_ = nh_.advertise<std_msgs::Empty>(heartbeat_topic_, 10);

    exec_timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(1.0, exec_rate_)),
                                  &PlannerFSM::execFSMCallback,
                                  this);
    safety_timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(1.0, safety_rate_)),
                                    &PlannerFSM::checkCollisionCallback,
                                    this);
    if (debug_map_publish_rate_ > 0.0)
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
    nh.param<std::string>("/local_planner/minco_trajectory_topic", minco_trajectory_topic_, minco_trajectory_topic_);
    nh.param<std::string>("/local_planner/heartbeat_topic", heartbeat_topic_, heartbeat_topic_);
    nh.param<double>("/local_planner/fsm/exec_rate", exec_rate_, exec_rate_);
    nh.param<double>("/local_planner/fsm/safety_rate", safety_rate_, safety_rate_);
    nh.param<double>("/local_planner/fsm/collision_check_step", collision_check_step_, collision_check_step_);
    nh.param<double>("/local_planner/debug_map_publish_rate", debug_map_publish_rate_, debug_map_publish_rate_);
    nh.param<bool>("/local_planner/fsm/have_trigger", have_trigger_, have_trigger_);

    pnh.param<std::string>("odom_topic", odom_topic_, odom_topic_);
    pnh.param<std::string>("cloud_topic", cloud_topic_, cloud_topic_);
    pnh.param<std::string>("goal_topic", goal_topic_, goal_topic_);
    pnh.param<std::string>("extra_goal_topic", extra_goal_topic_, extra_goal_topic_);
    pnh.param<std::string>("trigger_topic", trigger_topic_, trigger_topic_);
    pnh.param<std::string>("minco_trajectory_topic", minco_trajectory_topic_, minco_trajectory_topic_);
    pnh.param<std::string>("heartbeat_topic", heartbeat_topic_, heartbeat_topic_);
    pnh.param<double>("fsm/exec_rate", exec_rate_, exec_rate_);
    pnh.param<double>("fsm/safety_rate", safety_rate_, safety_rate_);
    pnh.param<double>("fsm/collision_check_step", collision_check_step_, collision_check_step_);
    pnh.param<double>("debug_map_publish_rate", debug_map_publish_rate_, debug_map_publish_rate_);
    pnh.param<bool>("fsm/have_trigger", have_trigger_, have_trigger_);
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

    init_pt_ = have_odom_ ? odom_pos_ : init_pt_;
    end_pt_ = Eigen::Vector3d(msg->pose.position.x,
                              msg->pose.position.y,
                              msg->pose.position.z);
    end_vel_.setZero();
    local_target_pt_ = end_pt_;
    local_target_vel_ = end_vel_;
    wps_.clear();
    wps_.push_back(end_pt_);
    current_wp_ = 0;
    have_target_ = true;
    have_new_target_ = true;
    request_replan_ = true;

    ROS_INFO("[FastNav][PlannerFSM] Goal received: [%.2f, %.2f, %.2f] in frame %s.",
             end_pt_.x(), end_pt_.y(), end_pt_.z(), planner_manager_->frameId().c_str());

    if (exec_state_ == WAIT_TARGET)
    {
        changeFSMExecState(GEN_NEW_TRAJ, "GOAL");
    }
    else if (exec_state_ == EXEC_TRAJ)
    {
        changeFSMExecState(REPLAN_TRAJ, "GOAL");
    }
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
        if (planFromGlobalTraj(1))
        {
            changeFSMExecState(EXEC_TRAJ, "FSM");
        }
        else
        {
            changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        }
        break;

    case REPLAN_TRAJ:
        if (planFromCurrentTraj(1))
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

    if (planner_manager_->isCurrentPathCollisionFree(collision_check_step_))
    {
        return;
    }

    ROS_WARN("[FastNav][PlannerFSM] Current path is in collision. Try replan from current state.");
    if (planFromCurrentTraj(1))
    {
        changeFSMExecState(EXEC_TRAJ, "SAFETY");
        return;
    }

    changeFSMExecState(EMERGENCY_STOP, "SAFETY");
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

    for (int i = 0; i < std::max(1, trial_times); ++i)
    {
        const bool use_random_init = i > 0 || continuously_called_times_ > 1;
        if (callReplan(false, use_random_init))
        {
            return true;
        }
    }

    return false;
}

// 从当前状态重新规划；当前局部轨迹由 manager 内部 MINCO 数据维护，重规划起点仍取最新 odom。
bool PlannerFSM::planFromCurrentTraj(int trial_times)
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

    for (int i = 0; i < std::max(1, trial_times); ++i)
    {
        if (callReplan(true, i > 0))
        {
            return true;
        }
    }

    return false;
}

// 统一规划入口：检查前置条件，选择局部目标，调用 manager 的 A* + PathOptimizer，并发布结果。
bool PlannerFSM::callReplan(bool use_current_path, bool use_random_init)
{
    if (!have_odom_ || !have_map_ || !have_target_)
    {
        ROS_WARN_THROTTLE(1.0,
                          "[FastNav][PlannerFSM] Cannot plan yet. odom=%d, map=%d, target=%d",
                          have_odom_, have_map_, have_target_);
        return false;
    }

    getLocalTarget();
    LocalPlannerManager::ReplanOptions options;
    options.use_current_traj = use_current_path;
    options.use_random_init = use_random_init;
    options.attempt = continuously_called_times_;
    options.continuous_failures = continuously_called_times_;

    const bool success = planner_manager_->planToGoal(local_target_pt_, options);
    searched_nodes_pub_.publish(planner_manager_->getSearchedNodesCloud());

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

// 当前没有全局轨迹切片逻辑，局部目标先直接等于最终目标；后续可在这里加入 planning horizon。
void PlannerFSM::getLocalTarget()
{
    local_target_pt_ = end_pt_;
    local_target_vel_ = end_vel_;
}

// 发布当前路径和 A* 搜索节点。路径消息仍保持 nav_msgs::Path，内容来自 MINCO 采样点或几何回退路径。
void PlannerFSM::publishTrajectory()
{
    path_pub_.publish(planner_manager_->getPathMsg());
    searched_nodes_pub_.publish(planner_manager_->getSearchedNodesCloud());
    publishMincoTrajectory();
}

// 发布 planner 内部维护的 occupied / inflated debug cloud，用于和独立 mapping 节点输出做对照。
void PlannerFSM::publishPlannerOutputs()
{
    if (!planner_manager_->hasMap())
    {
        return;
    }

    debug_occupied_pub_.publish(planner_manager_->getDebugOccupiedCloud());
    debug_inflated_pub_.publish(planner_manager_->getDebugInflatedCloud());
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

void PlannerFSM::publishHeartbeat()
{
    if (!heartbeat_pub_)
    {
        return;
    }

    std_msgs::Empty heartbeat;
    heartbeat_pub_.publish(heartbeat);
}

}  // namespace fastnav_planner
