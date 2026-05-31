#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <geometry_msgs/TransformStamped.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// cloud_filter_node 是感知链路的第二步：输入已经在 odom 下的点云 $P_o = {p_i}$，输出更干净、更稀疏的点云 $P_f$。
// 第一版只做预处理：NaN 去除、距离裁剪、空间裁剪、VoxelGrid 降采样，不做建图和多帧融合。
class CloudFilterNode
{
public:
    CloudFilterNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
        : nh_(nh),
          pnh_(pnh),
          // 滤波节点查询 $odom <- mid360_link$ 是为了拿到雷达在 odom 下的位置 $s$，用于距离裁剪 $d_i = ||p_i - s||$。
          tf_buffer_(),
          tf_listener_(tf_buffer_),
          // 输入点云已经由 cloud_transform_node 转成 odom 坐标系。
          input_topic_("/fastnav/perception/cloud_odom"),
          // 输出点云供 RViz、后续 local voxel map 和 obstacle inflation 使用。
          output_topic_("/fastnav/perception/cloud_filtered"),
          // cloud_frame_ 是点云坐标和裁剪盒所在坐标系，第一阶段为 $odom$。
          cloud_frame_("odom"),
          // sensor_frame_ 是雷达坐标系，用于查询雷达原点 $s$。
          sensor_frame_("mid360_link"),
          tf_timeout_(0.05),
          // 距离裁剪保留 $range_min <= ||p_i - s|| <= range_max$ 的点。
          range_min_(0.2),
          range_max_(20.0),
          // 空间裁剪盒在 odom 下定义，保留局部范围内的点。
          crop_x_min_(-20.0),
          crop_x_max_(20.0),
          crop_y_min_(-20.0),
          crop_y_max_(20.0),
          crop_z_min_(-3.0),
          crop_z_max_(5.0),
          // VoxelGrid 体素边长 $l$ 越大，点云越稀疏，计算越轻；$l$ 越小，几何细节保留越多。
          voxel_leaf_size_(0.1)
    {
        loadParameters();

        // 发布滤波后的点云 $P_f$。
        cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_, 5);
        // 订阅 odom 点云 $P_o$，每帧进入 cloudCallback() 完成预处理。
        cloud_sub_ = nh_.subscribe(input_topic_, 5,
                                   &CloudFilterNode::cloudCallback,
                                   this);

        ROS_INFO("[FastNav][CloudFilter] input: %s", input_topic_.c_str());
        ROS_INFO("[FastNav][CloudFilter] output: %s", output_topic_.c_str());
        ROS_INFO("[FastNav][CloudFilter] range: [%.2f, %.2f], voxel: %.3f",
                 range_min_, range_max_, voxel_leaf_size_);
    }

private:
    void loadParameters()
    {
        // 读取全局 yaml 参数 $/cloud_filter/*$，这些参数来自 perception.yaml。
        nh_.param<std::string>("/cloud_filter/input_topic", input_topic_, input_topic_);
        nh_.param<std::string>("/cloud_filter/output_topic", output_topic_, output_topic_);
        nh_.param<std::string>("/cloud_filter/cloud_frame", cloud_frame_, cloud_frame_);
        nh_.param<std::string>("/cloud_filter/sensor_frame", sensor_frame_, sensor_frame_);
        nh_.param<double>("/cloud_filter/tf_timeout", tf_timeout_, tf_timeout_);
        nh_.param<double>("/cloud_filter/range_min", range_min_, range_min_);
        nh_.param<double>("/cloud_filter/range_max", range_max_, range_max_);
        nh_.param<double>("/cloud_filter/crop_x_min", crop_x_min_, crop_x_min_);
        nh_.param<double>("/cloud_filter/crop_x_max", crop_x_max_, crop_x_max_);
        nh_.param<double>("/cloud_filter/crop_y_min", crop_y_min_, crop_y_min_);
        nh_.param<double>("/cloud_filter/crop_y_max", crop_y_max_, crop_y_max_);
        nh_.param<double>("/cloud_filter/crop_z_min", crop_z_min_, crop_z_min_);
        nh_.param<double>("/cloud_filter/crop_z_max", crop_z_max_, crop_z_max_);
        nh_.param<double>("/cloud_filter/voxel_leaf_size", voxel_leaf_size_, voxel_leaf_size_);

        // 私有参数作为覆盖入口，方便未来用 launch 为某个实验临时设置 $range_max$ 或 $voxel_leaf_size$。
        pnh_.param<std::string>("input_topic", input_topic_, input_topic_);
        pnh_.param<std::string>("output_topic", output_topic_, output_topic_);
        pnh_.param<std::string>("cloud_frame", cloud_frame_, cloud_frame_);
        pnh_.param<std::string>("sensor_frame", sensor_frame_, sensor_frame_);
        pnh_.param<double>("tf_timeout", tf_timeout_, tf_timeout_);
        pnh_.param<double>("range_min", range_min_, range_min_);
        pnh_.param<double>("range_max", range_max_, range_max_);
        pnh_.param<double>("crop_x_min", crop_x_min_, crop_x_min_);
        pnh_.param<double>("crop_x_max", crop_x_max_, crop_x_max_);
        pnh_.param<double>("crop_y_min", crop_y_min_, crop_y_min_);
        pnh_.param<double>("crop_y_max", crop_y_max_, crop_y_max_);
        pnh_.param<double>("crop_z_min", crop_z_min_, crop_z_min_);
        pnh_.param<double>("crop_z_max", crop_z_max_, crop_z_max_);
        pnh_.param<double>("voxel_leaf_size", voxel_leaf_size_, voxel_leaf_size_);
    }

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        // 四个点云变量对应处理流水线：$P_in -> P_finite -> P_clip -> P_voxel$。
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr finite_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr clipped_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        // ROS PointCloud2 是通用消息格式，PCL 点云方便逐点访问和滤波，因此先做格式转换。
        pcl::fromROSMsg(*msg, *input_cloud);

        // 去除非法点：只保留满足 $isfinite(x_i) && isfinite(y_i) && isfinite(z_i)$ 的点。
        std::vector<int> finite_indices;
        pcl::removeNaNFromPointCloud(*input_cloud, *finite_cloud, finite_indices);

        // 查询雷达在 odom 下的原点 $s = [s_x, s_y, s_z]^T$，距离裁剪用的是相对雷达距离而不是相对 odom 原点距离。
        const geometry_msgs::Vector3 sensor_origin = lookupSensorOrigin(msg->header.stamp);

        clipped_cloud->reserve(finite_cloud->size());
        for (const auto& point : finite_cloud->points)
        {
            // 空间裁剪盒条件为 $x_min <= x_i <= x_max$、$y_min <= y_i <= y_max$、$z_min <= z_i <= z_max$。
            if (!insideCropBox(point))
            {
                continue;
            }

            // 计算点到雷达原点的距离 $d_i = sqrt((x_i - s_x)^2 + (y_i - s_y)^2 + (z_i - s_z)^2)$。
            const double dx = point.x - sensor_origin.x;
            const double dy = point.y - sensor_origin.y;
            const double dz = point.z - sensor_origin.z;
            const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

            // 距离裁剪去掉过近噪声和过远低价值点，保留 $range_min <= d_i <= range_max$。
            if (distance < range_min_ || distance > range_max_)
            {
                continue;
            }

            clipped_cloud->push_back(point);
        }

        // 手动整理 PCL 点云元信息：这里输出的是无组织点云，因此 $height = 1$，$width = N$。
        clipped_cloud->width = clipped_cloud->size();
        clipped_cloud->height = 1;
        clipped_cloud->is_dense = true;

        if (voxel_leaf_size_ > 1e-6 && !clipped_cloud->empty())
        {
            // VoxelGrid 将空间按体素边长 $l$ 离散化，点 $p_i$ 会落入索引 $v_i = floor(p_i / l)$ 对应的体素。
            // 同一体素中的多个点被一个代表点近似，从而得到更稀疏的点集 $P_voxel$。
            pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
            voxel_filter.setInputCloud(clipped_cloud);
            voxel_filter.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
            voxel_filter.filter(*filtered_cloud);
        }
        else
        {
            // 若 $l <= 0$ 或当前没有点，则跳过体素滤波，直接输出裁剪结果。
            *filtered_cloud = *clipped_cloud;
        }

        // 将 PCL 点云转回 ROS PointCloud2，并保证输出坐标系仍为 $odom$。
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*filtered_cloud, output);
        output.header.stamp = msg->header.stamp;
        output.header.frame_id = cloud_frame_;

        cloud_pub_.publish(output);

        ROS_INFO_THROTTLE(1.0,
                          "[FastNav][CloudFilter] points: in=%zu, clipped=%zu, out=%zu",
                          input_cloud->size(),
                          clipped_cloud->size(),
                          filtered_cloud->size());
    }

    geometry_msgs::Vector3 lookupSensorOrigin(const ros::Time& stamp)
    {
        // 默认值为 odom 原点 $s = [0, 0, 0]^T$，仅在 TF 不可用时作为退化方案。
        geometry_msgs::Vector3 origin;
        origin.x = 0.0;
        origin.y = 0.0;
        origin.z = 0.0;

        try
        {
            // 查询 $cloud_frame <- sensor_frame$，若 cloud_frame 是 odom，则平移项就是雷达原点 $s$ 在 odom 下的位置。
            const geometry_msgs::TransformStamped transform =
                tf_buffer_.lookupTransform(cloud_frame_,
                                           sensor_frame_,
                                           stamp,
                                           ros::Duration(tf_timeout_));
            origin.x = transform.transform.translation.x;
            origin.y = transform.transform.translation.y;
            origin.z = transform.transform.translation.z;
        }
        catch (const tf2::TransformException& ex)
        {
            // TF 短暂缺失时不终止节点，只提示并使用默认 $s = [0, 0, 0]^T$ 继续处理当前帧。
            ROS_WARN_THROTTLE(1.0,
                              "[FastNav][CloudFilter] TF unavailable %s <- %s, using odom origin for range filter: %s",
                              cloud_frame_.c_str(),
                              sensor_frame_.c_str(),
                              ex.what());
        }

        return origin;
    }

    bool insideCropBox(const pcl::PointXYZ& point) const
    {
        // 局部裁剪盒是 odom 下的轴对齐长方体，数学条件为 $x_min <= x <= x_max$ 且 $y_min <= y <= y_max$ 且 $z_min <= z <= z_max$。
        return point.x >= crop_x_min_ && point.x <= crop_x_max_ &&
               point.y >= crop_y_min_ && point.y <= crop_y_max_ &&
               point.z >= crop_z_min_ && point.z <= crop_z_max_;
    }

private:
    // nh_ 用于全局 topic，pnh_ 用于私有参数覆盖。
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber cloud_sub_;
    ros::Publisher cloud_pub_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // 这些字符串参数定义第一版感知预处理的输入、输出和坐标系。
    std::string input_topic_;
    std::string output_topic_;
    std::string cloud_frame_;
    std::string sensor_frame_;
    double tf_timeout_;

    // 滤波参数：距离裁剪 $range_min <= d <= range_max$，空间裁剪盒，以及体素边长 $l$。
    double range_min_;
    double range_max_;
    double crop_x_min_;
    double crop_x_max_;
    double crop_y_min_;
    double crop_y_max_;
    double crop_z_min_;
    double crop_z_max_;
    double voxel_leaf_size_;
};

int main(int argc, char** argv)
{
    // ROS 节点名为 cloud_filter_node，对应 perception.launch 中的第二个感知节点。
    ros::init(argc, argv, "cloud_filter_node");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // 构造节点后开始订阅 $/fastnav/perception/cloud_odom$，并持续发布 $/fastnav/perception/cloud_filtered$。
    CloudFilterNode node(nh, pnh);

    ros::spin();
    return 0;
}
