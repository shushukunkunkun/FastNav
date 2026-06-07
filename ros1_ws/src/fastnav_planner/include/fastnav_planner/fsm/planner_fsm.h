/*
 * planner_fsm.h
 *
 * 本文件声明 PlannerFSM 类。
 * PlannerFSM 是 FastNav planner 的 ROS 入口和显式状态机，负责把外部事件组织成规划流程：
 * 1. 订阅 /fastnav/state/odom、/fastnav/perception/cloud_filtered、目标点 topic 和可选 trigger topic；
 * 2. 维护 INIT、WAIT_TARGET、GEN_NEW_TRAJ、REPLAN_TRAJ、EXEC_TRAJ、EMERGENCY_STOP 等状态；
 * 3. 持有 LocalPlannerManager::Ptr，把地图维护、A* 搜索、路径优化交给 manager；
 * 4. 发布 path、searched nodes、planner 内部 occupied / inflated debug cloud；
 * 5. 对齐 EGO-Planner 的 FSM 接口，保留 planFromGlobalTraj()、planFromCurrentTraj()、
 *    callReplan()、callEmergencyStop()、checkCollisionCallback() 等入口；
 * 6. 保持 FastNav 自己的轨迹表达，当前内部使用 MINCO，输出仍保留 nav_msgs::Path 便于 RViz 调试。
 *
 * 类内属性按用途分为：
 * - ROS 通信对象：Subscriber、Publisher、Timer；
 * - 算法组织对象：planner_manager_；
 * - FSM 状态变量：exec_state_、have_odom_、have_map_、have_target_ 等；
 * - 参数与 topic 名称：odom_topic_、cloud_topic_、goal_topic_、exec_rate_ 等。
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Empty.h>
#include <traj_utils/MincoTrajectory.h>

#include "fastnav_planner/manager/local_planner_manager.h"

namespace fastnav_planner
{

// PlannerFSM 是 FastNav planner 的显式状态机。
// 它只负责状态转换、ROS 输入输出和调用 planner_manager，不构造控制量，也不直接实现 A* / 地图算法。
class PlannerFSM
{
public:
    // FSMExecState 描述 planner 当前处于哪个规划阶段。
    enum FSMExecState
    {
        INIT,             // 等待 odom / 基础模块就绪。
        WAIT_TARGET,      // odom 和地图已就绪，等待目标点或触发信号。
        GEN_NEW_TRAJ,     // 从当前状态和新目标生成一条新路径。
        REPLAN_TRAJ,      // 基于当前目标重新规划，通常由新障碍或执行中重规划触发。
        EXEC_TRAJ,       // 已经发布可执行路径，等待控制层执行，同时监控是否需要重规划。
        EMERGENCY_STOP   // 紧急停止状态；当前版本保留入口，控制层负责最终停止动作。
    };

    // 初始化 FSM：读取参数、创建 manager、建立 ROS 通信对象和定时器。
    void init(ros::NodeHandle& nh, ros::NodeHandle& pnh);

private:
    // 读取 FSM 自身的 topic、频率、触发方式等参数。
    void loadParameters(ros::NodeHandle& nh, ros::NodeHandle& pnh);

    // ROS odom 回调：更新 have_odom_ 并转交 manager 作为地图中心和规划起点来源。
    void odometryCallback(const nav_msgs::OdometryConstPtr& msg);

    // ROS cloud 回调：更新 manager 内部 VoxelMap，并根据策略设置重规划请求。
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);

    // ROS goal 回调：保存最终目标点，触发新规划或执行中重规划。
    void waypointCallback(const geometry_msgs::PoseStampedConstPtr& msg);

    // ROS trigger 回调：真实实验或预设任务中用于显式允许 planner 开始工作。
    void triggerCallback(const geometry_msgs::PoseStampedConstPtr& msg);

    // FSM 主循环 timer：根据当前状态和前置条件执行状态转移。
    void execFSMCallback(const ros::TimerEvent& event);

    // 安全检查 timer：检查当前路径是否被新地图截断，必要时触发重规划或急停。
    void checkCollisionCallback(const ros::TimerEvent& event);

    // debug map timer：固定频率发布 planner 内部 VoxelMap 的可视化点云。
    void debugMapCallback(const ros::TimerEvent& event);

    // 切换 FSM 状态，并记录连续进入同一状态的次数。
    void changeFSMExecState(FSMExecState new_state, const std::string& caller);

    // 将状态枚举转换为字符串，便于日志输出。
    std::string stateName(FSMExecState state) const;

    // 周期性打印当前状态和关键前置条件。
    void printFSMExecState() const;

    // EGO 风格 helper：从当前全局目标生成新的局部路径。
    bool planFromGlobalTraj(int trial_times = 1);

    // EGO 风格 helper：从当前执行路径 / 当前 odom 重新规划到目标。
    bool planFromCurrentTraj(int trial_times = 1);

    // EGO 风格 helper：统一调用 manager 执行具体规划，当前底层是 A* + PathOptimizer。
    bool callReplan(bool use_current_path, bool use_random_init);

    // EGO 风格 helper：生成急停路径；当前 path 表达下先发布停在当前位置的单点路径。
    bool callEmergencyStop(const Eigen::Vector3d& stop_pos);

    // 根据全局目标选择局部目标；当前第一版没有全局轨迹，局部目标直接等于 end_pt_。
    void getLocalTarget();

    // 发布 manager 当前路径和 A* 搜索节点。
    void publishTrajectory();

    // 发布 planner 内部 occupied / inflated debug cloud。
    void publishPlannerOutputs();

    // 发布当前 MINCO 轨迹消息；traj_utils/minco_traj_server 会按时间采样成控制指令。
    void publishMincoTrajectory();

    // 发布 planner heartbeat，traj_server 用它判断 planner 是否仍然存活。
    void publishHeartbeat();

private:
    // ROS 句柄。nh_ 访问全局命名空间，pnh_ 访问节点私有命名空间。
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    // ROS 输入：里程计、点云、RViz 目标点和 FastNav 自定义目标点。
    ros::Subscriber odom_sub_;
    ros::Subscriber cloud_sub_;
    ros::Subscriber goal_sub_;
    ros::Subscriber extra_goal_sub_;
    ros::Subscriber trigger_sub_;

    // ROS 输出：路径、A* 搜索节点、planner 内部地图可视化。
    ros::Publisher path_pub_;
    ros::Publisher searched_nodes_pub_;
    ros::Publisher debug_occupied_pub_;
    ros::Publisher debug_inflated_pub_;
    ros::Publisher minco_traj_pub_;
    ros::Publisher heartbeat_pub_;

    // 定时器：FSM 主循环、安全检查和 debug map 发布。
    ros::Timer exec_timer_;
    ros::Timer safety_timer_;
    ros::Timer debug_map_timer_;

    // manager 持有 VoxelMap、AStarPlanner 和 PathOptimizer，FSM 只调用其接口。
    LocalPlannerManager::Ptr planner_manager_;

    // FSM 当前状态以及连续进入同一状态的次数，用于后续失败重试策略扩展。
    FSMExecState exec_state_{INIT};
    int continuously_called_times_{0};

    // planning data: FSM 前置条件和事件标志。
    // have_recv_pre_agent_ 属于 EGO 多机顺序启动逻辑，当前 FastNav 单机版本不保留。
    bool have_odom_{false};
    bool have_map_{false};
    bool have_target_{false};
    bool have_trigger_{true};
    bool have_new_target_{false};
    bool request_replan_{false};

    // odometry state: 当前无人机位置、速度、估计加速度和姿态。
    // nav_msgs::Odometry 不直接提供线加速度，这里用相邻速度差近似 $a_k=(v_k-v_{k-1})/\Delta t$。
    Eigen::Vector3d odom_pos_{0.0, 0.0, 0.0};
    Eigen::Vector3d odom_vel_{0.0, 0.0, 0.0};
    Eigen::Vector3d odom_acc_{0.0, 0.0, 0.0};
    Eigen::Quaterniond odom_orient_{Eigen::Quaterniond::Identity()};
    ros::Time last_odom_stamp_;
    Eigen::Vector3d last_odom_vel_{0.0, 0.0, 0.0};

    // 与 EGO-Planner 命名对齐的规划状态。当前 path planner 暂不强依赖速度 / 加速度约束，
    // 但这些字段是后续 MINCO / time-parameterized trajectory 的通用边界状态接口。
    Eigen::Vector3d init_pt_{0.0, 0.0, 0.0};
    Eigen::Vector3d start_pt_{0.0, 0.0, 0.0};
    Eigen::Vector3d start_vel_{0.0, 0.0, 0.0};
    Eigen::Vector3d start_acc_{0.0, 0.0, 0.0};
    Eigen::Vector3d start_yaw_{0.0, 0.0, 0.0};
    Eigen::Vector3d end_pt_{0.0, 0.0, 0.0};
    Eigen::Vector3d end_vel_{0.0, 0.0, 0.0};
    Eigen::Vector3d local_target_pt_{0.0, 0.0, 0.0};
    Eigen::Vector3d local_target_vel_{0.0, 0.0, 0.0};

    // waypoint data: 后续支持多航点任务时，wps_ 保存整段任务航点，current_wp_ 表示当前目标航点编号。
    std::vector<Eigen::Vector3d> wps_;
    int current_wp_{0};

    // emergency data: 后续若需要从近障状态主动逃离，可用该标志区分普通急停和逃逸式重规划。
    bool flag_escape_emergency_{false};

    // ROS topic 参数。
    std::string odom_topic_{"/fastnav/state/odom"};
    std::string cloud_topic_{"/fastnav/perception/cloud_filtered"};
    std::string goal_topic_{"/move_base_simple/goal"};
    std::string extra_goal_topic_{"/fastnav/planner/goal"};
    std::string trigger_topic_{"/traj_start_trigger"};
    std::string path_topic_{"/fastnav/planner/path"};
    std::string searched_nodes_topic_{"/fastnav/planner/searched_nodes"};
    std::string debug_occupied_cloud_topic_{"/fastnav/planner/debug_occupied_cloud"};
    std::string debug_inflated_cloud_topic_{"/fastnav/planner/debug_inflated_cloud"};
    std::string minco_trajectory_topic_{"/fastnav/planner/minco_trajectory"};
    std::string heartbeat_topic_{"/fastnav/planner/heartbeat"};

    // FSM 运行频率和行为开关。
    double exec_rate_{20.0};
    double safety_rate_{10.0};
    double debug_map_publish_rate_{5.0};
    double collision_check_step_{0.1};
};

}  // namespace fastnav_planner
