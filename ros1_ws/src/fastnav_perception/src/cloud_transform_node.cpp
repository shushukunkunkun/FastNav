#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

// cloud_transform_node 是感知链路的第一步：把雷达坐标系下的点云变换到 FastNav 内部统一坐标系 odom。
// 对单个点而言，若输入点为 $p_l = [x_l, y_l, z_l]^T$，TF 给出旋转 $R$ 和平移 $t$，输出点为 $p_o = R p_l + t$。
class CloudTransformNode
{
public:
    CloudTransformNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
        : nh_(nh),
          pnh_(pnh),
          // tf_buffer_ 缓存 TF 树，tf_listener_ 持续接收 /tf 和 /tf_static，使节点可以查询 $T_odom_mid360$。
          tf_buffer_(),
          tf_listener_(tf_buffer_),
          // 默认输入为 FastNav 标准雷达接口，要求点云 header.frame_id 通常为 $mid360_link$。
          input_topic_("/fastnav/lidar/points"),
          // 默认输出是 odom 点云，后续滤波节点只消费这个 topic，不再关心雷达安装坐标系。
          output_topic_("/fastnav/perception/cloud_odom"),
          // target_frame_ 是转换目标坐标系，第一阶段固定为 $odom$。
          target_frame_("odom"),
          // tf_timeout_ 是单帧点云等待 TF 的最长时间，避免 TF 暂时缺失时阻塞整个感知链路。
          tf_timeout_(0.05)
    {
        loadParameters();

        // 发布转换后的点云 $P_o = {p_o^i}$，队列长度 5 用于吸收短时间调度抖动。
        cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_, 5);
        // 订阅原始雷达点云 $P_l = {p_l^i}$，每帧到达后在 cloudCallback() 中整体变换到 odom。
        cloud_sub_ = nh_.subscribe(input_topic_, 5,
                                   &CloudTransformNode::cloudCallback,
                                   this);

        ROS_INFO("[FastNav][CloudTransform] input: %s", input_topic_.c_str());
        ROS_INFO("[FastNav][CloudTransform] output: %s", output_topic_.c_str());
        ROS_INFO("[FastNav][CloudTransform] target frame: %s", target_frame_.c_str());
    }

private:
    void loadParameters()
    {
        // 先读取全局 yaml 参数 $/cloud_transform/*$，这对应 perception.launch 中 rosparam 的加载方式。
        nh_.param<std::string>("/cloud_transform/input_topic",
                               input_topic_,
                               input_topic_);
        nh_.param<std::string>("/cloud_transform/output_topic",
                               output_topic_,
                               output_topic_);
        nh_.param<std::string>("/cloud_transform/target_frame",
                               target_frame_,
                               target_frame_);
        nh_.param<double>("/cloud_transform/tf_timeout",
                          tf_timeout_,
                          tf_timeout_);

        // 再读取节点私有参数，便于未来用 roslaunch 的 <param> 覆盖单个节点配置。
        pnh_.param<std::string>("input_topic", input_topic_, input_topic_);
        pnh_.param<std::string>("output_topic", output_topic_, output_topic_);
        pnh_.param<std::string>("target_frame", target_frame_, target_frame_);
        pnh_.param<double>("tf_timeout", tf_timeout_, tf_timeout_);
    }

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        // 没有 frame_id 就无法确定输入点 $p_l$ 属于哪个坐标系，因此不能计算 $p_o = R p_l + t$。
        if (msg->header.frame_id.empty())
        {
            ROS_WARN_THROTTLE(1.0,
                              "[FastNav][CloudTransform] Input cloud has empty frame_id.");
            return;
        }

        geometry_msgs::TransformStamped transform;
        try
        {
            // 查询目标坐标系到输入坐标系的变换，即 $target_frame <- msg->header.frame_id$。
            // 如果 target_frame 是 $odom$ 且输入是 $mid360_link$，这里得到的就是把雷达点变到 odom 的 $T_odom_mid360$。
            transform = tf_buffer_.lookupTransform(
                target_frame_,
                msg->header.frame_id,
                msg->header.stamp,
                ros::Duration(tf_timeout_));
        }
        catch (const tf2::TransformException& ex)
        {
            // TF 树在启动初期可能还没有连通，例如 $odom -> base_link -> mid360_link$ 尚未全部发布。
            // 这里丢弃当前帧而不是崩溃，等待下一帧点云和新的 TF。
            ROS_WARN_THROTTLE(1.0,
                              "[FastNav][CloudTransform] TF unavailable %s <- %s: %s",
                              target_frame_.c_str(),
                              msg->header.frame_id.c_str(),
                              ex.what());
            return;
        }

        sensor_msgs::PointCloud2 output;
        try
        {
            // tf2_sensor_msgs 会遍历 PointCloud2 中的每个点，并对每个点应用 $p_o = R p_l + t$。
            // 该操作只改变点的坐标表达，不改变同一物理点在空间中的真实位置。
            tf2::doTransform(*msg, output, transform);
        }
        catch (const tf2::TransformException& ex)
        {
            ROS_WARN_THROTTLE(1.0,
                              "[FastNav][CloudTransform] Failed to transform cloud: %s",
                              ex.what());
            return;
        }

        // 沿用原始点云时间戳 $t_k$，表示这帧 odom 点云仍然对应同一时刻的雷达扫描。
        output.header.stamp = msg->header.stamp;
        // 强制输出 frame_id 为目标坐标系，保证下游看到的点云满足 $frame_id = odom$。
        output.header.frame_id = target_frame_;
        cloud_pub_.publish(output);
    }

private:
    // nh_ 用于全局 topic 订阅发布，pnh_ 用于读取节点私有参数。
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber cloud_sub_;
    ros::Publisher cloud_pub_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // input_topic_、output_topic_、target_frame_ 共同定义转换链路 $P_l -> P_o$。
    std::string input_topic_;
    std::string output_topic_;
    std::string target_frame_;
    double tf_timeout_;
};

int main(int argc, char** argv)
{
    // ROS 节点名为 cloud_transform_node，对应 perception.launch 中的第一个感知节点。
    ros::init(argc, argv, "cloud_transform_node");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // 构造节点后开始订阅点云；之后 ros::spin() 让回调函数持续处理每一帧 $P_l$。
    CloudTransformNode node(nh, pnh);

    ros::spin();
    return 0;
}
