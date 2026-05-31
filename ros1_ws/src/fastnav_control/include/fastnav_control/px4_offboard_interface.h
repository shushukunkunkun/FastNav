// 该接口封装机载算法与 PX4/MAVROS 之间的 Offboard 通信细节。
#pragma once

#include <string>

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>

#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>

namespace fastnav_control
{

class Px4OffboardInterface
{
public:
    // 从私有参数读取 MAVROS 话题/服务名，并初始化订阅、发布和服务客户端。
    Px4OffboardInterface(ros::NodeHandle& nh, ros::NodeHandle& pnh);

    // 当前 MAVROS 与飞控的连接和解锁状态。
    bool isConnected() const;
    bool isArmed() const;

    // 是否已经收到至少一帧本地里程计，用于避免控制逻辑读取未初始化位置。
    bool hasOdom() const;

    // PX4 当前模式，例如 MANUAL、POSCTL、OFFBOARD。
    std::string getMode() const;

    // 返回最近一次 MAVROS 状态和本地里程计快照。
    mavros_msgs::State getState() const;
    nav_msgs::Odometry getOdom() const;

    // 通过 MAVROS 服务请求进入 PX4 OFFBOARD 模式。
    bool setOffboardMode();

    // 通过 MAVROS 解锁/上锁飞控。
    bool arm();
    bool disarm();

    // 发布位置 setpoint。x/y/z 使用 MAVROS local ENU 坐标系，yaw 单位为弧度。
    void publishPositionSetpoint(double x,
                                 double y,
                                 double z,
                                 double yaw = 0.0);

    // 发布速度 setpoint。线速度使用 MAVROS local ENU 坐标系，yaw_rate 单位为弧度/秒。
    void publishVelocitySetpoint(double vx,
                                 double vy,
                                 double vz,
                                 double yaw_rate = 0.0);

private:
    // MAVROS 回调仅缓存最新状态，控制逻辑通过 getter 读取快照。
    void stateCallback(const mavros_msgs::State::ConstPtr& msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

    // 将平面偏航角转换成 geometry_msgs 四元数，roll/pitch 固定为 0。
    geometry_msgs::Quaternion yawToQuaternion(double yaw) const;

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Subscriber state_sub_;
    ros::Subscriber odom_sub_;

    ros::Publisher position_setpoint_pub_;
    ros::Publisher velocity_setpoint_pub_;

    ros::ServiceClient arming_client_;
    ros::ServiceClient set_mode_client_;

    mavros_msgs::State current_state_;
    nav_msgs::Odometry current_odom_;

    bool has_odom_;

    std::string state_topic_;
    std::string odom_topic_;
    std::string position_setpoint_topic_;
    std::string velocity_setpoint_topic_;

    std::string arming_service_;
    std::string set_mode_service_;
};

}  // namespace fastnav_control
