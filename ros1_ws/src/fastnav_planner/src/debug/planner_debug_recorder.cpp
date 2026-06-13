#include "fastnav_planner/debug/planner_debug_recorder.h"

#include <cerrno>
#include <cmath>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/console.h>
#include <ros/time.h>

namespace fastnav_planner
{

namespace
{

std::string sanitizeToken(const std::string& text)
{
    std::string out;
    out.reserve(text.size());
    for (char c : text)
    {
        if ((c >= 'a' && c <= 'z') ||
            (c >= 'A' && c <= 'Z') ||
            (c >= '0' && c <= '9') ||
            c == '_' || c == '-')
        {
            out.push_back(c);
        }
        else
        {
            out.push_back('_');
        }
    }
    return out.empty() ? "unknown" : out;
}

std::string vecToYaml(const Eigen::Vector3d& v)
{
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(6)
       << "[" << v.x() << ", " << v.y() << ", " << v.z() << "]";
    return ss.str();
}

}  // namespace

void PlannerDebugRecorder::setConfig(const Config& config)
{
    config_ = config;
    config_.minco_sample_dt = std::max(1.0e-3, config_.minco_sample_dt);
    config_.max_cases = std::max(0, config_.max_cases);
}

bool PlannerDebugRecorder::recordMincoFailure(const Snapshot& snapshot)
{
    if (!config_.enable)
    {
        return false;
    }
    if (config_.max_cases > 0 && recorded_cases_ >= config_.max_cases)
    {
        ROS_WARN_THROTTLE(5.0,
                          "[FastNav][PlannerDebugRecorder] Reached max_cases=%d, skip failure snapshot.",
                          config_.max_cases);
        return false;
    }

    const std::string case_dir = createCaseDirectory(snapshot);
    if (case_dir.empty())
    {
        return false;
    }

    bool ok = true;
    ok = writeMeta(case_dir, snapshot) && ok;
    ok = writePathCsv(case_dir + "/frontend_path.csv", snapshot.frontend_path) && ok;
    ok = writePathCsv(case_dir + "/optimization_reference_path.csv", snapshot.reference_path) && ok;
    ok = writePathCsv(case_dir + "/shortcut_path.csv", snapshot.shortcut_path) && ok;
    ok = writePathCsv(case_dir + "/sampled_path.csv", snapshot.sampled_path) && ok;
    ok = writePathCsv(case_dir + "/searched_nodes.csv", snapshot.searched_nodes) && ok;

    if (config_.record_corridor)
    {
        ok = writeCorridorCsv(case_dir + "/safe_corridor.csv", snapshot.corridors) && ok;
    }

    if (config_.record_minco_samples && snapshot.has_minco && snapshot.minco_traj.valid())
    {
        ok = writeMincoSamplesCsv(case_dir + "/minco_samples.csv",
                                  snapshot.minco_traj,
                                  config_.minco_sample_dt) && ok;
    }

    if (config_.record_cloud && snapshot.has_filtered_cloud)
    {
        ok = writeCloudPcd(case_dir + "/cloud_filtered.pcd", snapshot.filtered_cloud) && ok;
    }
    if (config_.record_voxel_map && snapshot.has_occupied_cloud)
    {
        ok = writeCloudPcd(case_dir + "/debug_occupied_cloud.pcd", snapshot.occupied_cloud) && ok;
    }
    if (config_.record_voxel_map && snapshot.has_inflated_cloud)
    {
        ok = writeCloudPcd(case_dir + "/debug_inflated_cloud.pcd", snapshot.inflated_cloud) && ok;
    }

    if (ok)
    {
        ++recorded_cases_;
        last_case_dir_ = case_dir;
        ROS_WARN("[FastNav][PlannerDebugRecorder] Saved MINCO failure snapshot: %s",
                 case_dir.c_str());
    }
    else
    {
        ROS_WARN("[FastNav][PlannerDebugRecorder] Snapshot partially saved: %s",
                 case_dir.c_str());
    }
    return ok;
}

bool PlannerDebugRecorder::ensureDirectory(const std::string& path) const
{
    if (path.empty())
    {
        return false;
    }

    std::string normalized = path;
    while (normalized.size() > 1 && normalized.back() == '/')
    {
        normalized.pop_back();
    }

    size_t pos = normalized.front() == '/' ? 1 : 0;
    while (true)
    {
        pos = normalized.find('/', pos);
        const std::string partial = pos == std::string::npos
                                        ? normalized
                                        : normalized.substr(0, pos);
        if (!partial.empty())
        {
            if (::mkdir(partial.c_str(), 0755) != 0 && errno != EEXIST)
            {
                ROS_WARN("[FastNav][PlannerDebugRecorder] mkdir failed: %s", partial.c_str());
                return false;
            }
        }
        if (pos == std::string::npos)
        {
            break;
        }
        ++pos;
    }
    return true;
}

std::string PlannerDebugRecorder::createCaseDirectory(const Snapshot& snapshot)
{
    if (!ensureDirectory(config_.output_dir))
    {
        return std::string();
    }

    const ros::WallTime wall_now = ros::WallTime::now();
    const double now_sec = wall_now.toSec();
    const std::time_t whole_sec = static_cast<std::time_t>(std::floor(now_sec));
    const int millis = static_cast<int>(std::round((now_sec - std::floor(now_sec)) * 1000.0));

    std::tm tm_buf;
    localtime_r(&whole_sec, &tm_buf);

    std::ostringstream ss;
    ss << config_.output_dir << "/fail_"
       << std::put_time(&tm_buf, "%Y%m%d_%H%M%S")
       << "_" << std::setw(3) << std::setfill('0') << millis
       << "_" << std::setw(4) << std::setfill('0') << recorded_cases_
       << "_" << sanitizeToken(snapshot.state);

    const std::string case_dir = ss.str();
    return ensureDirectory(case_dir) ? case_dir : std::string();
}

bool PlannerDebugRecorder::writeMeta(const std::string& dir,
                                     const Snapshot& snapshot) const
{
    std::ofstream out(dir + "/meta.yaml");
    if (!out.is_open())
    {
        return false;
    }

    out << std::fixed << std::setprecision(6);
    out << "stamp: " << snapshot.stamp.toSec() << "\n";
    out << "frame_id: \"" << snapshot.frame_id << "\"\n";
    out << "state: \"" << snapshot.state << "\"\n";
    out << "reason: \"" << snapshot.reason << "\"\n";
    out << "attempt: " << snapshot.attempt << "\n";
    out << "continuous_failures: " << snapshot.continuous_failures << "\n";
    out << "use_current_traj: " << (snapshot.use_current_traj ? "true" : "false") << "\n";
    out << "use_random_init: " << (snapshot.use_random_init ? "true" : "false") << "\n";
    out << "touch_goal: " << (snapshot.touch_goal ? "true" : "false") << "\n";
    out << "reached_requested_goal: " << (snapshot.reached_requested_goal ? "true" : "false") << "\n";
    out << "odom_pos: " << vecToYaml(snapshot.odom_pos) << "\n";
    out << "start: " << vecToYaml(snapshot.start) << "\n";
    out << "requested_goal: " << vecToYaml(snapshot.requested_goal) << "\n";
    out << "planned_target: " << vecToYaml(snapshot.planned_target) << "\n";
    out << "counts:\n";
    out << "  frontend_path: " << snapshot.frontend_path.size() << "\n";
    out << "  reference_path: " << snapshot.reference_path.size() << "\n";
    out << "  shortcut_path: " << snapshot.shortcut_path.size() << "\n";
    out << "  sampled_path: " << snapshot.sampled_path.size() << "\n";
    out << "  searched_nodes: " << snapshot.searched_nodes.size() << "\n";
    out << "  corridors: " << snapshot.corridors.size() << "\n";
    out << "  has_minco: " << (snapshot.has_minco ? "true" : "false") << "\n";
    out << "timing_ms:\n";
    out << "  guide_astar: " << snapshot.timing.guide_astar_ms << "\n";
    out << "  frontend_astar: " << snapshot.timing.frontend_astar_ms << "\n";
    out << "  reference: " << snapshot.timing.reference_ms << "\n";
    out << "  shortcut: " << snapshot.timing.shortcut_ms << "\n";
    out << "  corridor: " << snapshot.timing.corridor_ms << "\n";
    out << "  minco: " << snapshot.timing.minco_ms << "\n";
    out << "  fine_check: " << snapshot.timing.fine_check_ms << "\n";
    out << "planner_stats:\n";
    out << "  astar_nodes: " << snapshot.timing.astar_nodes << "\n";
    out << "  corridor_num: " << snapshot.timing.corridor_num << "\n";
    out << "  minco_retry_count: " << snapshot.timing.minco_retry_count << "\n";
    out << "  clearance_used: " << snapshot.timing.clearance_used << "\n";
    return true;
}

bool PlannerDebugRecorder::writePathCsv(const std::string& path,
                                        const std::vector<Eigen::Vector3d>& points) const
{
    std::ofstream out(path);
    if (!out.is_open())
    {
        return false;
    }

    out << "index,x,y,z\n";
    out << std::fixed << std::setprecision(8);
    for (size_t i = 0; i < points.size(); ++i)
    {
        out << i << ","
            << points[i].x() << ","
            << points[i].y() << ","
            << points[i].z() << "\n";
    }
    return true;
}

bool PlannerDebugRecorder::writeCorridorCsv(const std::string& path,
                                            const std::vector<Eigen::MatrixX4d>& corridors) const
{
    std::ofstream out(path);
    if (!out.is_open())
    {
        return false;
    }

    out << "corridor_id,plane_id,nx,ny,nz,d\n";
    out << std::fixed << std::setprecision(8);
    for (size_t i = 0; i < corridors.size(); ++i)
    {
        const Eigen::MatrixX4d& corridor = corridors[i];
        for (int r = 0; r < corridor.rows(); ++r)
        {
            out << i << ","
                << r << ","
                << corridor(r, 0) << ","
                << corridor(r, 1) << ","
                << corridor(r, 2) << ","
                << corridor(r, 3) << "\n";
        }
    }
    return true;
}

bool PlannerDebugRecorder::writeMincoSamplesCsv(const std::string& path,
                                                const fastnav::MincoTraj& traj,
                                                double sample_dt) const
{
    if (!traj.valid())
    {
        return false;
    }

    std::ofstream out(path);
    if (!out.is_open())
    {
        return false;
    }

    const double step = std::max(1.0e-3, sample_dt);
    const double duration = traj.getDuration();
    const int sample_num = std::max(1, static_cast<int>(std::ceil(duration / step)));

    out << "t,px,py,pz,vx,vy,vz,ax,ay,az,jx,jy,jz\n";
    out << std::fixed << std::setprecision(8);
    for (int i = 0; i <= sample_num; ++i)
    {
        const double t = std::min(duration, static_cast<double>(i) * step);
        const Eigen::Vector3d p = traj.getPosition(t);
        const Eigen::Vector3d v = traj.getVelocity(t);
        const Eigen::Vector3d a = traj.getAcceleration(t);
        const Eigen::Vector3d j = traj.getJerk(t);
        out << t << ","
            << p.x() << "," << p.y() << "," << p.z() << ","
            << v.x() << "," << v.y() << "," << v.z() << ","
            << a.x() << "," << a.y() << "," << a.z() << ","
            << j.x() << "," << j.y() << "," << j.z() << "\n";
    }
    return true;
}

bool PlannerDebugRecorder::writeCloudPcd(const std::string& path,
                                         const sensor_msgs::PointCloud2& cloud) const
{
    if (cloud.width == 0 || cloud.data.empty())
    {
        return false;
    }

    pcl::PointCloud<pcl::PointXYZ> xyz_cloud;
    pcl::fromROSMsg(cloud, xyz_cloud);
    return pcl::io::savePCDFileBinary(path, xyz_cloud) == 0;
}

}  // namespace fastnav_planner
