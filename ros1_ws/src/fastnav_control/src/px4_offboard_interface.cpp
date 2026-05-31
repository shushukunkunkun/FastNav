#include "fastnav_control/px4_offboard_interface.h"

#include <cmath>

namespace fastnav_control
{

Px4OffboardInterface::Px4OffboardInterface(ros::NodeHandle& nh,
                                           ros::NodeHandle& pnh)
    : nh_(nh),
      pnh_(pnh),
      has_odom_(false)
{
    // 允许通过私有参数覆盖 MAVROS 默认话题/服务名，便于仿真和真机配置复用。
    pnh_.param<std::string>("topic/state",
                            state_topic_,
                            "/mavros/state");

    pnh_.param<std::string>("topic/odom",
                            odom_topic_,
                            "/mavros/local_position/odom");

    pnh_.param<std::string>("topic/position_setpoint",
                            position_setpoint_topic_,
                            "/mavros/setpoint_position/local");

    pnh_.param<std::string>("topic/velocity_setpoint",
                            velocity_setpoint_topic_,
                            "/mavros/setpoint_velocity/cmd_vel");

    pnh_.param<std::string>("service/arming",
                            arming_service_,
                            "/mavros/cmd/arming");

    pnh_.param<std::string>("service/set_mode",
                            set_mode_service_,
                            "/mavros/set_mode");

    state_sub_ = nh_.subscribe<mavros_msgs::State>(
        state_topic_, 10, &Px4OffboardInterface::stateCallback, this);

    odom_sub_ = nh_.subscribe<nav_msgs::Odometry>(
        odom_topic_, 10, &Px4OffboardInterface::odomCallback, this);

    position_setpoint_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
        position_setpoint_topic_, 10);

    velocity_setpoint_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(
        velocity_setpoint_topic_, 10);

    arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>(
        arming_service_);

    // set_mode 使用 custom_mode 字符串请求 PX4 模式，例如 OFFBOARD。
    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>(
        set_mode_service_);

    ROS_INFO("[FastNav][Px4OffboardInterface] Initialized.");
    ROS_INFO("[FastNav][Px4OffboardInterface] state topic: %s",
             state_topic_.c_str());
    ROS_INFO("[FastNav][Px4OffboardInterface] odom topic: %s",
             odom_topic_.c_str());
}

bool Px4OffboardInterface::isConnected() const // NOTE: 这个在函数后面加上 const 是为了保证这个函数不会修改类的成员变量或者调用其他非 const 成员函数
{
    return current_state_.connected;
}

bool Px4OffboardInterface::isArmed() const
{
    return current_state_.armed;
}

bool Px4OffboardInterface::hasOdom() const
{
    return has_odom_;
}

std::string Px4OffboardInterface::getMode() const
{
    return current_state_.mode;
}

mavros_msgs::State Px4OffboardInterface::getState() const
{
    return current_state_;
}

nav_msgs::Odometry Px4OffboardInterface::getOdom() const
{
    return current_odom_;
}

bool Px4OffboardInterface::setOffboardMode()
{
    mavros_msgs::SetMode set_mode;
    set_mode.request.custom_mode = "OFFBOARD";

    // call() 失败表示 ROS 服务通信失败；mode_sent=false 表示飞控拒绝了模式请求。
    if (!set_mode_client_.call(set_mode))
    {
        ROS_WARN("[FastNav][Px4OffboardInterface] Failed to call set_mode service.");
        return false;
    }

    if (!set_mode.response.mode_sent)
    {
        ROS_WARN("[FastNav][Px4OffboardInterface] OFFBOARD mode request rejected.");
        return false;
    }

    ROS_INFO("[FastNav][Px4OffboardInterface] OFFBOARD mode request sent.");
    return true;
}

bool Px4OffboardInterface::arm()
{
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    // success=false 通常意味着 PX4 预解锁检查未通过或当前状态不允许解锁。
    if (!arming_client_.call(arm_cmd))
    {
        ROS_WARN("[FastNav][Px4OffboardInterface] Failed to call arming service.");
        return false;
    }

    if (!arm_cmd.response.success)
    {
        ROS_WARN("[FastNav][Px4OffboardInterface] Arm request rejected.");
        return false;
    }

    ROS_INFO("[FastNav][Px4OffboardInterface] Arm request accepted.");
    return true;
}

bool Px4OffboardInterface::disarm()
{
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = false;

    // 上锁与解锁共用 MAVROS CommandBool 服务，只通过 request.value 区分。
    if (!arming_client_.call(arm_cmd))
    {
        ROS_WARN("[FastNav][Px4OffboardInterface] Failed to call disarming service.");
        return false;
    }

    if (!arm_cmd.response.success)
    {
        ROS_WARN("[FastNav][Px4OffboardInterface] Disarm request rejected.");
        return false;
    }

    ROS_INFO("[FastNav][Px4OffboardInterface] Disarm request accepted.");
    return true;
}

void Px4OffboardInterface::publishPositionSetpoint(double x,
                                                   double y,
                                                   double z,
                                                   double yaw)
{
    geometry_msgs::PoseStamped pose_msg;

    // MAVROS local position setpoint 通常使用 ENU 局部坐标，frame_id 与 odom 保持一致。
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = "map";

    pose_msg.pose.position.x = x;
    pose_msg.pose.position.y = y;
    pose_msg.pose.position.z = z;
    pose_msg.pose.orientation = yawToQuaternion(yaw);

    position_setpoint_pub_.publish(pose_msg);
}

void Px4OffboardInterface::publishVelocitySetpoint(double vx,
                                                   double vy,
                                                   double vz,
                                                   double yaw_rate)
{
    geometry_msgs::TwistStamped twist_msg;

    // 速度 setpoint 只填充线速度和绕 z 轴偏航角速度，其余角速度保持默认 0。
    twist_msg.header.stamp = ros::Time::now();
    twist_msg.header.frame_id = "map";

    twist_msg.twist.linear.x = vx;
    twist_msg.twist.linear.y = vy;
    twist_msg.twist.linear.z = vz;
    twist_msg.twist.angular.z = yaw_rate;

    velocity_setpoint_pub_.publish(twist_msg);
}

void Px4OffboardInterface::stateCallback(const mavros_msgs::State::ConstPtr& msg)
{
    current_state_ = *msg;
}

void Px4OffboardInterface::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    current_odom_ = *msg;
    // 记录首帧里程计是否到达，调用方可据此决定是否开始控制。
    has_odom_ = true;
}

geometry_msgs::Quaternion Px4OffboardInterface::yawToQuaternion(double yaw) const
{
    geometry_msgs::Quaternion q;

    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(yaw * 0.5);
    q.w = std::cos(yaw * 0.5);

    return q;
}

}  // namespace fastnav_control
