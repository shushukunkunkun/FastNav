#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>

class FastNavOffboardNode
{
public:
    FastNavOffboardNode()
    {
        state_sub_ = nh_.subscribe<mavros_msgs::State>(
            "/mavros/state", 10, &FastNavOffboardNode::stateCallback, this);

        odom_sub_ = nh_.subscribe<nav_msgs::Odometry>(
            "/mavros/local_position/odom", 10, &FastNavOffboardNode::odomCallback, this);

        local_pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
            "/mavros/setpoint_position/local", 10);

        arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>(
            "/mavros/cmd/arming");

        set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>(
            "/mavros/set_mode");

        target_pose_.pose.position.x = 0.0;
        target_pose_.pose.position.y = 0.0;
        target_pose_.pose.position.z = 1.5;

        target_pose_.pose.orientation.w = 1.0;
    }

    void run()
    {
        ros::Rate rate(20.0);

        ROS_INFO("[FastNav] Waiting for FCU connection...");

        while (ros::ok() && !current_state_.connected)
        {
            ros::spinOnce();
            rate.sleep();
        }

        ROS_INFO("[FastNav] FCU connected.");

        /*
         * PX4 Offboard 模式要求在切换 OFFBOARD 前，
         * 已经持续发送一段时间 setpoint。
         */
        for (int i = 0; ros::ok() && i < 100; ++i)
        {
            target_pose_.header.stamp = ros::Time::now();
            local_pos_pub_.publish(target_pose_);

            ros::spinOnce();
            rate.sleep();
        }

        mavros_msgs::SetMode offboard_set_mode;
        offboard_set_mode.request.custom_mode = "OFFBOARD";

        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = true;

        ros::Time last_request = ros::Time::now();

        ROS_INFO("[FastNav] Start Offboard takeoff and hover.");

        while (ros::ok())
        {
            target_pose_.header.stamp = ros::Time::now();
            local_pos_pub_.publish(target_pose_);

            if (current_state_.mode != "OFFBOARD" &&
                (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                if (set_mode_client_.call(offboard_set_mode) &&
                    offboard_set_mode.response.mode_sent)
                {
                    ROS_INFO("[FastNav] OFFBOARD mode enabled.");
                }

                last_request = ros::Time::now();
            }
            else
            {
                if (!current_state_.armed &&
                    (ros::Time::now() - last_request > ros::Duration(5.0)))
                {
                    if (arming_client_.call(arm_cmd) &&
                        arm_cmd.response.success)
                    {
                        ROS_INFO("[FastNav] Vehicle armed.");
                    }

                    last_request = ros::Time::now();
                }
            }

            ROS_INFO_THROTTLE(
                1.0,
                "[FastNav] mode: %s, armed: %d, pos: [%.2f, %.2f, %.2f]",
                current_state_.mode.c_str(),
                current_state_.armed,
                current_odom_.pose.pose.position.x,
                current_odom_.pose.pose.position.y,
                current_odom_.pose.pose.position.z);

            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    void stateCallback(const mavros_msgs::State::ConstPtr& msg)
    {
        current_state_ = *msg;
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        current_odom_ = *msg;
    }

private:
    ros::NodeHandle nh_;

    ros::Subscriber state_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher local_pos_pub_;

    ros::ServiceClient arming_client_;
    ros::ServiceClient set_mode_client_;

    mavros_msgs::State current_state_;
    nav_msgs::Odometry current_odom_;
    geometry_msgs::PoseStamped target_pose_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fastnav_offboard_node");

    FastNavOffboardNode node;
    node.run();

    return 0;
}