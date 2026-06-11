/*
 * FastNav MINCO trajectory server.
 *
 * 这个节点效仿 EGO-Planner-v2 的 traj_server：planner 只发布完整轨迹，
 * traj_server 负责按 ROS 时间采样 $p(t),v(t),a(t),j(t),\psi(t),\dot\psi(t)$，
 * 再把采样结果发布为 fastnav_msgs::ControlCommand。
 *
 * 这样可以把“规划轨迹”和“控制指令”解耦：
 * - planner 不直接构造 PX4 setpoint；
 * - controller 不理解 MINCO 系数，只消费期望状态；
 * - 后续若轨迹表达从 MINCO 换成 B-spline，只需要替换 traj_server 的采样逻辑。
 */

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Eigen>
#include <fastnav_msgs/ControlCommand.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <traj_utils/MincoTrajectory.h>
#include <traj_utils/minco/gcopter/trajectory.hpp>

namespace
{
using MincoTrajectory = Trajectory<5>;
using MincoPiece = Piece<5>;

ros::Publisher cmd_pub;
ros::Subscriber traj_sub;
ros::Subscriber heartbeat_sub;
ros::Timer cmd_timer;

struct ServerTrajectory
{
    std::shared_ptr<MincoTrajectory> traj;
    ros::Time start_time;
    double duration{0.0};
    uint32_t traj_id{0};
    std::string frame_id{"odom"};
    bool touch_goal{true};

    bool valid() const
    {
        return static_cast<bool>(traj) && duration > 1.0e-6;
    }
};

ServerTrajectory current_traj;
ServerTrajectory pending_traj;
bool has_current_traj = false;
bool has_pending_traj = false;
ros::Time heartbeat_time(0);
std::string frame_id = "odom";

double publish_rate = 100.0;
double heartbeat_timeout = 3.0;
double terminal_hold_timeout = 0.8;
double time_forward = 1.0;
double yaw_dot_limit = 2.0 * M_PI;
double yaw_ddot_limit = 5.0 * M_PI;
double default_yaw = 0.0;

Eigen::Vector3d last_pos = Eigen::Vector3d::Zero();
double last_yaw = 0.0;
double last_yaw_dot = 0.0;

bool buildTrajectory(const traj_utils::MincoTrajectory& msg, MincoTrajectory& output)
{
    constexpr int kOrder = 5;
    constexpr int kCoeffNum = kOrder + 1;

    if (msg.order != kOrder)
    {
        ROS_ERROR("[FastNav][MincoTrajServer] Only support MINCO order 5, got %d.", msg.order);
        return false;
    }

    const size_t piece_num = msg.duration.size();
    if (piece_num == 0 ||
        msg.coef_x.size() != piece_num * kCoeffNum ||
        msg.coef_y.size() != piece_num * kCoeffNum ||
        msg.coef_z.size() != piece_num * kCoeffNum)
    {
        ROS_ERROR("[FastNav][MincoTrajServer] Invalid MINCO coefficient size.");
        return false;
    }

    std::vector<double> durations(piece_num);
    std::vector<MincoPiece::CoefficientMat> coeff_mats(piece_num);

    for (size_t i = 0; i < piece_num; ++i)
    {
        durations[i] = msg.duration[i];
        if (durations[i] <= 1.0e-6)
        {
            ROS_ERROR("[FastNav][MincoTrajServer] Piece %zu duration is invalid: %.6f.", i, durations[i]);
            return false;
        }

        for (int k = 0; k < kCoeffNum; ++k)
        {
            const size_t id = i * kCoeffNum + k;
            coeff_mats[i](0, k) = msg.coef_x[id];
            coeff_mats[i](1, k) = msg.coef_y[id];
            coeff_mats[i](2, k) = msg.coef_z[id];
        }
    }

    output = MincoTrajectory(durations, coeff_mats);
    return output.getPieceNum() > 0 && output.getTotalDuration() > 1.0e-6;
}

std::pair<double, double> calculateYaw(const ServerTrajectory& active,
                                       double t_cur,
                                       const Eigen::Vector3d& pos,
                                       double dt)
{
    if (!active.valid())
    {
        return std::make_pair(default_yaw, 0.0);
    }

    const double lookahead_t = std::min(active.duration, t_cur + std::max(0.0, time_forward));
    const Eigen::Vector3d dir = active.traj->getPos(lookahead_t) - pos;
    double target_yaw = dir.head<2>().norm() > 0.1 ? std::atan2(dir.y(), dir.x()) : last_yaw;

    double d_yaw = target_yaw - last_yaw;
    if (d_yaw > M_PI)
    {
        d_yaw -= 2.0 * M_PI;
    }
    if (d_yaw < -M_PI)
    {
        d_yaw += 2.0 * M_PI;
    }

    const double signed_yaw_dot_limit = d_yaw >= 0.0 ? yaw_dot_limit : -yaw_dot_limit;
    const double signed_yaw_ddot_limit = d_yaw >= 0.0 ? yaw_ddot_limit : -yaw_ddot_limit;

    double max_delta_yaw = 0.0;
    if (std::fabs(last_yaw_dot + dt * signed_yaw_ddot_limit) <= std::fabs(signed_yaw_dot_limit))
    {
        max_delta_yaw = last_yaw_dot * dt + 0.5 * signed_yaw_ddot_limit * dt * dt;
    }
    else
    {
        const double t1 = (signed_yaw_dot_limit - last_yaw_dot) / signed_yaw_ddot_limit;
        max_delta_yaw = ((dt - t1) + dt) * (signed_yaw_dot_limit - last_yaw_dot) * 0.5;
    }

    if (std::fabs(d_yaw) > std::fabs(max_delta_yaw))
    {
        d_yaw = max_delta_yaw;
    }

    double yaw = last_yaw + d_yaw;
    if (yaw > M_PI)
    {
        yaw -= 2.0 * M_PI;
    }
    if (yaw < -M_PI)
    {
        yaw += 2.0 * M_PI;
    }

    const double yaw_dot = dt > 1.0e-4 ? d_yaw / dt : 0.0;
    last_yaw = yaw;
    last_yaw_dot = yaw_dot;

    return std::make_pair(yaw, yaw_dot);
}

void publishCommand(const ros::Time& stamp,
                    const std::string& command_frame_id,
                    uint32_t command_traj_id,
                    const Eigen::Vector3d& pos,
                    const Eigen::Vector3d& vel,
                    const Eigen::Vector3d& acc,
                    const Eigen::Vector3d& jerk,
                    double yaw,
                    double yaw_rate,
                    uint8_t command_type)
{
    fastnav_msgs::ControlCommand cmd;
    cmd.header.stamp = stamp;
    cmd.header.frame_id = command_frame_id;
    cmd.command_type = command_type;
    cmd.trajectory_id = command_traj_id;
    cmd.position.x = pos.x();
    cmd.position.y = pos.y();
    cmd.position.z = pos.z();
    cmd.velocity.x = vel.x();
    cmd.velocity.y = vel.y();
    cmd.velocity.z = vel.z();
    cmd.acceleration.x = acc.x();
    cmd.acceleration.y = acc.y();
    cmd.acceleration.z = acc.z();
    cmd.jerk.x = jerk.x();
    cmd.jerk.y = jerk.y();
    cmd.jerk.z = jerk.z();
    cmd.yaw = yaw;
    cmd.yaw_rate = yaw_rate;
    cmd.enable = true;

    cmd_pub.publish(cmd);
    last_pos = pos;
}

void heartbeatCallback(const std_msgs::EmptyConstPtr& /*msg*/)
{
    heartbeat_time = ros::Time::now();
}

bool heartbeatTimedOut(const ros::Time& now)
{
    return heartbeat_timeout > 0.0 &&
           heartbeat_time.toSec() > 1.0e-5 &&
           (now - heartbeat_time).toSec() > heartbeat_timeout;
}

ServerTrajectory makeServerTrajectory(const traj_utils::MincoTrajectory& msg,
                                      const MincoTrajectory& source_traj)
{
    ServerTrajectory output;
    output.traj.reset(new MincoTrajectory(source_traj));
    output.start_time = msg.start_time.isZero() ? msg.header.stamp : msg.start_time;
    if (output.start_time.isZero())
    {
        output.start_time = ros::Time::now();
    }
    output.duration = output.traj->getTotalDuration();
    output.traj_id = msg.traj_id;
    output.frame_id = msg.header.frame_id.empty() ? frame_id : msg.header.frame_id;
    output.touch_goal = msg.touch_goal;
    return output;
}

void commitTrajectory(const ServerTrajectory& next_traj, const std::string& reason)
{
    current_traj = next_traj;
    has_current_traj = current_traj.valid();
    frame_id = current_traj.frame_id;
    if (has_current_traj)
    {
        last_pos = current_traj.traj->getPos(0.0);
        ROS_INFO("[FastNav][MincoTrajServer] Commit trajectory id=%u, pieces=%d, duration=%.3f, touch_goal=%d (%s).",
                 current_traj.traj_id,
                 current_traj.traj->getPieceNum(),
                 current_traj.duration,
                 current_traj.touch_goal,
                 reason.c_str());
    }
}

void switchPendingTrajectoryIfReady(const ros::Time& now)
{
    if (!has_pending_traj || !pending_traj.valid())
    {
        return;
    }

    if (now + ros::Duration(1.0e-4) < pending_traj.start_time)
    {
        return;
    }

    commitTrajectory(pending_traj, "pending start time reached");
    has_pending_traj = false;
    pending_traj = ServerTrajectory();
}

void trajectoryCallback(const traj_utils::MincoTrajectoryConstPtr& msg)
{
    MincoTrajectory new_traj;
    if (!buildTrajectory(*msg, new_traj))
    {
        return;
    }

    const ros::Time now = ros::Time::now();
    ServerTrajectory incoming = makeServerTrajectory(*msg, new_traj);
    heartbeat_time = ros::Time::now();

    if (has_current_traj && incoming.start_time > now + ros::Duration(0.02))
    {
        pending_traj = incoming;
        has_pending_traj = true;
        ROS_INFO("[FastNav][MincoTrajServer] Received pending trajectory id=%u, starts in %.3fs, duration=%.3f, touch_goal=%d.",
                 pending_traj.traj_id,
                 (pending_traj.start_time - now).toSec(),
                 pending_traj.duration,
                 pending_traj.touch_goal);
        return;
    }

    commitTrajectory(incoming, "immediate trajectory message");
    has_pending_traj = false;
    pending_traj = ServerTrajectory();
}

void commandTimerCallback(const ros::TimerEvent& event)
{
    const ros::Time now = ros::Time::now();
    switchPendingTrajectoryIfReady(now);

    if (!has_current_traj || !current_traj.valid())
    {
        return;
    }

    const double t_cur = (now - current_traj.start_time).toSec();
    const double dt = std::max(1.0e-3, (event.current_real - event.last_real).toSec());

    if (t_cur < 0.0)
    {
        return;
    }

    if (t_cur <= current_traj.duration)
    {
        const Eigen::Vector3d pos = current_traj.traj->getPos(t_cur);
        const Eigen::Vector3d vel = current_traj.traj->getVel(t_cur);
        const Eigen::Vector3d acc = current_traj.traj->getAcc(t_cur);
        const Eigen::Vector3d jerk = current_traj.traj->getJer(t_cur);
        const std::pair<double, double> yaw = calculateYaw(current_traj, t_cur, pos, dt);

        publishCommand(now,
                       current_traj.frame_id,
                       current_traj.traj_id,
                       pos,
                       vel,
                       acc,
                       jerk,
                       yaw.first,
                       yaw.second,
                       fastnav_msgs::ControlCommand::COMMAND_TRAJECTORY);
        return;
    }

    const double over_time = t_cur - current_traj.duration;
    const Eigen::Vector3d end_pos = current_traj.traj->getPos(current_traj.duration);
    const Eigen::Vector3d end_vel = current_traj.traj->getVel(current_traj.duration);
    const Eigen::Vector3d end_acc = current_traj.traj->getAcc(current_traj.duration);
    const Eigen::Vector3d end_jerk = current_traj.traj->getJer(current_traj.duration);

    // committed trajectory 执行期间不因为 planner 正在阻塞式重规划而立刻 HOVER；
    // 只有旧轨迹已经结束后，heartbeat 仍丢失，才认为 planner 不再能接管后续轨迹。
    if (heartbeatTimedOut(now))
    {
        ROS_ERROR_THROTTLE(1.0, "[FastNav][MincoTrajServer] Lost planner heartbeat after committed trajectory ended, switch to hover command.");
        publishCommand(now,
                       current_traj.frame_id,
                       current_traj.traj_id,
                       end_pos,
                       Eigen::Vector3d::Zero(),
                       Eigen::Vector3d::Zero(),
                       Eigen::Vector3d::Zero(),
                       last_yaw,
                       0.0,
                       fastnav_msgs::ControlCommand::COMMAND_HOVER);
        has_current_traj = false;
        has_pending_traj = false;
        return;
    }

    if (!current_traj.touch_goal && over_time <= terminal_hold_timeout)
    {
        publishCommand(now,
                       current_traj.frame_id,
                       current_traj.traj_id,
                       end_pos,
                       end_vel,
                       end_acc,
                       end_jerk,
                       last_yaw,
                       0.0,
                       fastnav_msgs::ControlCommand::COMMAND_TRAJECTORY);
        return;
    }

    publishCommand(now,
                   current_traj.frame_id,
                   current_traj.traj_id,
                   end_pos,
                   Eigen::Vector3d::Zero(),
                   Eigen::Vector3d::Zero(),
                   Eigen::Vector3d::Zero(),
                   last_yaw,
                   0.0,
                   fastnav_msgs::ControlCommand::COMMAND_HOVER);
}
}  // namespace

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fastnav_minco_traj_server");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::string trajectory_topic = "/fastnav/planner/minco_trajectory";
    std::string heartbeat_topic = "/fastnav/planner/heartbeat";
    std::string command_topic = "/fastnav/control_cmd";

    pnh.param<std::string>("trajectory_topic", trajectory_topic, trajectory_topic);
    pnh.param<std::string>("heartbeat_topic", heartbeat_topic, heartbeat_topic);
    pnh.param<std::string>("command_topic", command_topic, command_topic);
    pnh.param<std::string>("frame_id", frame_id, frame_id);
    pnh.param<double>("publish_rate", publish_rate, publish_rate);
    pnh.param<double>("heartbeat_timeout", heartbeat_timeout, heartbeat_timeout);
    pnh.param<double>("terminal_hold_timeout", terminal_hold_timeout, terminal_hold_timeout);
    pnh.param<double>("time_forward", time_forward, time_forward);
    pnh.param<double>("yaw_dot_limit", yaw_dot_limit, yaw_dot_limit);
    pnh.param<double>("yaw_ddot_limit", yaw_ddot_limit, yaw_ddot_limit);
    pnh.param<double>("default_yaw", default_yaw, default_yaw);

    last_yaw = default_yaw;

    traj_sub = nh.subscribe(trajectory_topic, 10, trajectoryCallback);
    heartbeat_sub = nh.subscribe(heartbeat_topic, 10, heartbeatCallback);
    cmd_pub = nh.advertise<fastnav_msgs::ControlCommand>(command_topic, 20);
    cmd_timer = nh.createTimer(ros::Duration(1.0 / std::max(1.0, publish_rate)),
                               commandTimerCallback);

    ROS_INFO("[FastNav][MincoTrajServer] trajectory topic: %s", trajectory_topic.c_str());
    ROS_INFO("[FastNav][MincoTrajServer] command topic: %s", command_topic.c_str());
    ROS_INFO("[FastNav][MincoTrajServer] terminal hold timeout: %.3fs", terminal_hold_timeout);

    ros::spin();
    return 0;
}
