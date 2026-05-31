#include "fastnav_control/control_fsm.h"

#include <cmath>

namespace fastnav_control
{

ControlFSM::ControlFSM(ros::NodeHandle& nh,
                       ros::NodeHandle& pnh,
                       Px4OffboardInterface& px4_interface)
    : nh_(nh),
      pnh_(pnh),
      px4_(px4_interface),
      current_state_(State::WAIT_FOR_FCU),
      target_x_(0.0),
      target_y_(0.0),
      target_z_(1.5),
      target_yaw_(0.0),
      hover_x_(0.0),
      hover_y_(0.0),
      hover_z_(1.5),
      hover_yaw_(0.0),
      takeoff_position_tolerance_(0.15),
      init_setpoint_count_(100),
      init_setpoint_counter_(0),
      request_interval_(5.0),
      control_cmd_topic_("/fastnav/control_cmd"),
      control_cmd_timeout_(0.5),
      has_control_cmd_(false),
      last_control_cmd_time_(ros::Time(0)),
      last_request_time_(ros::Time(0)),
      state_start_time_(ros::Time::now())
{
    loadParameters();

    control_cmd_sub_ = nh_.subscribe<fastnav_msgs::ControlCommand>(
        control_cmd_topic_,
        10,
        &ControlFSM::controlCommandCallback,
        this);

    hover_x_ = target_x_;
    hover_y_ = target_y_;
    hover_z_ = target_z_;
    hover_yaw_ = target_yaw_;

    ROS_INFO("[FastNav][ControlFSM] Initialized.");
    ROS_INFO("[FastNav][ControlFSM] takeoff target: [%.2f, %.2f, %.2f], yaw: %.2f",
             target_x_, target_y_, target_z_, target_yaw_);
    ROS_INFO("[FastNav][ControlFSM] control command topic: %s",
             control_cmd_topic_.c_str());
}

void ControlFSM::loadParameters()
{
    pnh_.param<double>("takeoff/x", target_x_, 0.0);
    pnh_.param<double>("takeoff/y", target_y_, 0.0);
    pnh_.param<double>("takeoff/z", target_z_, 1.5);
    pnh_.param<double>("takeoff/yaw", target_yaw_, 0.0);

    pnh_.param<double>("takeoff/position_tolerance",
                       takeoff_position_tolerance_,
                       0.15);

    pnh_.param<int>("offboard/init_setpoint_count",
                    init_setpoint_count_,
                    100);

    pnh_.param<double>("offboard/request_interval",
                       request_interval_,
                       5.0);

    pnh_.param<std::string>("topic/control_cmd",
                            control_cmd_topic_,
                            "/fastnav/control_cmd");

    pnh_.param<double>("control_cmd/timeout",
                       control_cmd_timeout_,
                       0.5);
}

void ControlFSM::controlCommandCallback(const fastnav_msgs::ControlCommand::ConstPtr& msg)
{
    latest_control_cmd_ = *msg;
    has_control_cmd_ = true;
    last_control_cmd_time_ = ros::Time::now();
}

void ControlFSM::runOnce()
{
    switch (current_state_)
    {
    case State::WAIT_FOR_FCU:
        handleWaitForFcu();
        break;

    case State::WAIT_FOR_ODOM:
        handleWaitForOdom();
        break;

    case State::SEND_INIT_SETPOINT:
        handleSendInitSetpoint();
        break;

    case State::SET_OFFBOARD:
        handleSetOffboard();
        break;

    case State::ARM:
        handleArm();
        break;

    case State::TAKEOFF:
        handleTakeoff();
        break;

    case State::HOVER:
        handleHover();
        break;

    case State::COMMAND_CONTROL:
        handleCommandControl();
        break;

    default:
        ROS_WARN_THROTTLE(1.0, "[FastNav][ControlFSM] Unknown state.");
        break;
    }
}

void ControlFSM::handleWaitForFcu()
{
    ROS_INFO_THROTTLE(1.0, "[FastNav][ControlFSM] Waiting for FCU connection...");

    if (px4_.isConnected())
    {
        ROS_INFO("[FastNav][ControlFSM] FCU connected.");
        transitTo(State::WAIT_FOR_ODOM);
    }
}

void ControlFSM::handleWaitForOdom()
{
    ROS_INFO_THROTTLE(1.0, "[FastNav][ControlFSM] Waiting for odometry...");

    if (px4_.hasOdom())
    {
        ROS_INFO("[FastNav][ControlFSM] Odometry received.");
        transitTo(State::SEND_INIT_SETPOINT);
    }
}

void ControlFSM::handleSendInitSetpoint()
{
    publishTakeoffSetpoint();

    ++init_setpoint_counter_;

    ROS_INFO_THROTTLE(1.0,
                      "[FastNav][ControlFSM] Sending initial setpoints: %d / %d",
                      init_setpoint_counter_,
                      init_setpoint_count_);

    if (init_setpoint_counter_ >= init_setpoint_count_)
    {
        transitTo(State::SET_OFFBOARD);
    }
}

void ControlFSM::handleSetOffboard()
{
    publishTakeoffSetpoint();

    if (px4_.getMode() == "OFFBOARD")
    {
        ROS_INFO("[FastNav][ControlFSM] Already in OFFBOARD mode.");
        transitTo(State::ARM);
        return;
    }

    if (requestIntervalPassed())
    {
        ROS_INFO("[FastNav][ControlFSM] Requesting OFFBOARD mode...");

        px4_.setOffboardMode();
        last_request_time_ = ros::Time::now();
    }

    if (px4_.getMode() == "OFFBOARD")
    {
        transitTo(State::ARM);
    }
}

void ControlFSM::handleArm()
{
    publishTakeoffSetpoint();

    if (px4_.getMode() != "OFFBOARD")
    {
        ROS_WARN_THROTTLE(1.0,
                          "[FastNav][ControlFSM] Vehicle left OFFBOARD mode, requesting OFFBOARD again.");
        transitTo(State::SET_OFFBOARD);
        return;
    }

    if (px4_.isArmed())
    {
        ROS_INFO("[FastNav][ControlFSM] Vehicle already armed.");
        transitTo(State::TAKEOFF);
        return;
    }

    if (requestIntervalPassed())
    {
        ROS_INFO("[FastNav][ControlFSM] Requesting arm...");

        px4_.arm();
        last_request_time_ = ros::Time::now();
    }

    if (px4_.isArmed())
    {
        transitTo(State::TAKEOFF);
    }
}

void ControlFSM::handleTakeoff()
{
    publishTakeoffSetpoint();

    const nav_msgs::Odometry odom = px4_.getOdom();

    ROS_INFO_THROTTLE(1.0,
                      "[FastNav][ControlFSM] Taking off. Current z: %.2f, target z: %.2f",
                      odom.pose.pose.position.z,
                      target_z_);

    if (isTakeoffReached())
    {
        ROS_INFO("[FastNav][ControlFSM] Takeoff target reached.");
        updateHoverTargetFromOdom();
        transitTo(State::HOVER);
    }
}

void ControlFSM::handleHover()
{
    publishHoverSetpoint();

    const nav_msgs::Odometry odom = px4_.getOdom();

    ROS_INFO_THROTTLE(1.0,
                      "[FastNav][ControlFSM] Hovering. pos: [%.2f, %.2f, %.2f], mode: %s, armed: %d",
                      odom.pose.pose.position.x,
                      odom.pose.pose.position.y,
                      odom.pose.pose.position.z,
                      px4_.getMode().c_str(),
                      px4_.isArmed());

    if (px4_.getMode() != "OFFBOARD")
    {
        ROS_WARN_THROTTLE(1.0,
                          "[FastNav][ControlFSM] Vehicle is not in OFFBOARD during hover.");
        transitTo(State::SET_OFFBOARD);
        return;
    }

    if (hasFreshControlCommand() && latest_control_cmd_.enable)
    {
        if (latest_control_cmd_.command_type == fastnav_msgs::ControlCommand::COMMAND_IDLE)
        {
            return;
        }

        if (latest_control_cmd_.command_type == fastnav_msgs::ControlCommand::COMMAND_HOVER)
        {
            updateHoverTargetFromOdom();
            return;
        }

        ROS_INFO("[FastNav][ControlFSM] Valid control command received.");
        transitTo(State::COMMAND_CONTROL);
    }
}

void ControlFSM::handleCommandControl()
{
    if (px4_.getMode() != "OFFBOARD")
    {
        ROS_WARN_THROTTLE(1.0,
                          "[FastNav][ControlFSM] Vehicle left OFFBOARD mode during command control.");
        transitTo(State::SET_OFFBOARD);
        return;
    }

    if (!px4_.isArmed())
    {
        ROS_WARN_THROTTLE(1.0,
                          "[FastNav][ControlFSM] Vehicle is not armed during command control.");
        transitTo(State::ARM);
        return;
    }

    if (!hasFreshControlCommand() || !latest_control_cmd_.enable)
    {
        ROS_WARN_THROTTLE(1.0,
                          "[FastNav][ControlFSM] Control command timeout or disabled. Switching to hover.");
        updateHoverTargetFromOdom();
        transitTo(State::HOVER);
        return;
    }

    executeControlCommand(latest_control_cmd_);
}

void ControlFSM::executeControlCommand(const fastnav_msgs::ControlCommand& cmd)
{
    switch (cmd.command_type)
    {
    case fastnav_msgs::ControlCommand::COMMAND_POSITION:
        px4_.publishPositionSetpoint(cmd.position.x,
                                     cmd.position.y,
                                     cmd.position.z,
                                     cmd.yaw);

        ROS_INFO_THROTTLE(1.0,
                          "[FastNav][ControlFSM] POSITION cmd: [%.2f, %.2f, %.2f], yaw: %.2f",
                          cmd.position.x,
                          cmd.position.y,
                          cmd.position.z,
                          cmd.yaw);
        break;

    case fastnav_msgs::ControlCommand::COMMAND_VELOCITY:
        px4_.publishVelocitySetpoint(cmd.velocity.x,
                                     cmd.velocity.y,
                                     cmd.velocity.z,
                                     cmd.yaw_rate);

        ROS_INFO_THROTTLE(1.0,
                          "[FastNav][ControlFSM] VELOCITY cmd: [%.2f, %.2f, %.2f], yaw_rate: %.2f",
                          cmd.velocity.x,
                          cmd.velocity.y,
                          cmd.velocity.z,
                          cmd.yaw_rate);
        break;

    case fastnav_msgs::ControlCommand::COMMAND_HOVER:
        updateHoverTargetFromOdom();
        transitTo(State::HOVER);
        break;

    case fastnav_msgs::ControlCommand::COMMAND_LAND:
        ROS_WARN_THROTTLE(1.0,
                          "[FastNav][ControlFSM] LAND command received, but landing is not implemented yet.");
        publishHoverSetpoint();
        break;

    case fastnav_msgs::ControlCommand::COMMAND_IDLE:
        updateHoverTargetFromOdom();
        transitTo(State::HOVER);
        break;

    default:
        ROS_WARN_THROTTLE(1.0,
                          "[FastNav][ControlFSM] Unknown command type: %d",
                          cmd.command_type);
        publishHoverSetpoint();
        break;
    }
}

void ControlFSM::publishTakeoffSetpoint()
{
    px4_.publishPositionSetpoint(target_x_,
                                 target_y_,
                                 target_z_,
                                 target_yaw_);
}

void ControlFSM::publishHoverSetpoint()
{
    px4_.publishPositionSetpoint(hover_x_,
                                 hover_y_,
                                 hover_z_,
                                 hover_yaw_);
}

bool ControlFSM::isTakeoffReached() const
{
    if (!px4_.hasOdom())
    {
        return false;
    }

    const nav_msgs::Odometry odom = px4_.getOdom();

    const double dx = odom.pose.pose.position.x - target_x_;
    const double dy = odom.pose.pose.position.y - target_y_;
    const double dz = odom.pose.pose.position.z - target_z_;

    const double position_error = std::sqrt(dx * dx + dy * dy + dz * dz);

    return position_error < takeoff_position_tolerance_;
}

bool ControlFSM::requestIntervalPassed() const
{
    if (last_request_time_.isZero())
    {
        return true;
    }

    return (ros::Time::now() - last_request_time_).toSec() > request_interval_;
}

bool ControlFSM::hasFreshControlCommand() const
{
    if (!has_control_cmd_)
    {
        return false;
    }

    return (ros::Time::now() - last_control_cmd_time_).toSec() < control_cmd_timeout_;
}

void ControlFSM::updateHoverTargetFromOdom()
{
    if (!px4_.hasOdom())
    {
        hover_x_ = target_x_;
        hover_y_ = target_y_;
        hover_z_ = target_z_;
        hover_yaw_ = target_yaw_;
        return;
    }

    const nav_msgs::Odometry odom = px4_.getOdom();

    hover_x_ = odom.pose.pose.position.x;
    hover_y_ = odom.pose.pose.position.y;
    hover_z_ = odom.pose.pose.position.z;
    hover_yaw_ = target_yaw_;

    ROS_INFO("[FastNav][ControlFSM] Hover target updated: [%.2f, %.2f, %.2f]",
             hover_x_,
             hover_y_,
             hover_z_);
}

void ControlFSM::transitTo(State new_state)
{
    if (new_state == current_state_)
    {
        return;
    }

    ROS_INFO("[FastNav][ControlFSM] State transition: %s -> %s",
             stateToString(current_state_).c_str(),
             stateToString(new_state).c_str());

    current_state_ = new_state;
    state_start_time_ = ros::Time::now();

    if (new_state == State::SEND_INIT_SETPOINT)
    {
        init_setpoint_counter_ = 0;
    }
}

ControlFSM::State ControlFSM::getState() const
{
    return current_state_;
}

std::string ControlFSM::getStateName() const
{
    return stateToString(current_state_);
}

std::string ControlFSM::stateToString(State state) const
{
    switch (state)
    {
    case State::WAIT_FOR_FCU:
        return "WAIT_FOR_FCU";

    case State::WAIT_FOR_ODOM:
        return "WAIT_FOR_ODOM";

    case State::SEND_INIT_SETPOINT:
        return "SEND_INIT_SETPOINT";

    case State::SET_OFFBOARD:
        return "SET_OFFBOARD";

    case State::ARM:
        return "ARM";

    case State::TAKEOFF:
        return "TAKEOFF";

    case State::HOVER:
        return "HOVER";

    case State::COMMAND_CONTROL:
        return "COMMAND_CONTROL";

    default:
        return "UNKNOWN";
    }
}

}  // namespace fastnav_control