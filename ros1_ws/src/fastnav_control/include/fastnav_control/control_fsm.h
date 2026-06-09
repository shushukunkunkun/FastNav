#pragma once
#include <string>

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <fastnav_msgs/ControlCommand.h>

#include "fastnav_control/px4_offboard_interface.h"

namespace fastnav_control
{

class ControlFSM
{
public:
    enum class State
    {
        WAIT_FOR_FCU = 0,
        WAIT_FOR_ODOM,
        SEND_INIT_SETPOINT,
        SET_OFFBOARD,
        ARM,
        TAKEOFF,
        HOVER,
        COMMAND_CONTROL
    };

public:
    ControlFSM(ros::NodeHandle& nh,
               ros::NodeHandle& pnh,
               Px4OffboardInterface& px4_interface);

    void runOnce();

    State getState() const;
    std::string getStateName() const;

private:
    void loadParameters();

    void controlCommandCallback(const fastnav_msgs::ControlCommand::ConstPtr& msg);

    void handleWaitForFcu();
    void handleWaitForOdom();
    void handleSendInitSetpoint();
    void handleSetOffboard();
    void handleArm();
    void handleTakeoff();
    void handleHover();
    void handleCommandControl();

    void publishTakeoffSetpoint();
    void publishHoverSetpoint();

    void executeControlCommand(const fastnav_msgs::ControlCommand& cmd);

    bool isTakeoffReached() const;
    bool requestIntervalPassed() const;
    bool hasFreshControlCommand() const;

    void updateHoverTargetFromOdom();

    void transitTo(State new_state);
    std::string stateToString(State state) const;
    void publishFSMState();

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    Px4OffboardInterface& px4_;

    State current_state_;

    ros::Subscriber control_cmd_sub_;
    ros::Publisher fsm_state_pub_;

    double target_x_;
    double target_y_;
    double target_z_;
    double target_yaw_;

    double hover_x_;
    double hover_y_;
    double hover_z_;
    double hover_yaw_;

    double takeoff_position_tolerance_;

    int init_setpoint_count_;
    int init_setpoint_counter_;

    double request_interval_;

    std::string control_cmd_topic_;
    std::string fsm_state_topic_;
    double control_cmd_timeout_;

    bool has_control_cmd_;
    fastnav_msgs::ControlCommand latest_control_cmd_;
    ros::Time last_control_cmd_time_;

    ros::Time last_request_time_;
    ros::Time state_start_time_;
};

}  // namespace fastnav_control
