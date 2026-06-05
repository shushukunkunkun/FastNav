#ifndef _PLAN_CONTAINER_H_
#define _PLAN_CONTAINER_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <cmath>
#include <vector>
#include <string>
#include <ros/ros.h>

#include <traj_utils/uniform_bspline.h>
#include <traj_utils/polynomial_traj.h>
#include <traj_utils/minco/minco_traj.h>

using std::vector;

namespace ego_planner
{

  /*
   * GlobalTrajData 保存“全局参考轨迹”的状态。
   *
   * 这里的全局轨迹不是最终直接下发给控制器的轨迹，而是由起点到目标点/航点生成的
   * 粗略多项式参考线。EGOReplanFSM 会沿着这条参考线向前采样，选择当前 planning
   * horizon 内的局部目标点，然后交给 EGOPlannerManager 生成真正可执行的局部 B 样条。
   */
  class GlobalTrajData
  {
  private:
  public:
    PolynomialTraj global_traj_;          // 全局多项式参考轨迹，通常由 planGlobalTraj() 生成。
    vector<UniformBspline> local_traj_;   // 可选的局部 B 样条替换段：[0]位置、[1]速度、[2]加速度。

    double global_duration_;              // 当前全局参考轨迹的总时长。
    ros::Time global_start_time_;         // 全局轨迹生成时的 ROS 时间戳。
    double local_start_time_, local_end_time_; // 局部替换段在全局时间轴上的起止时间。
    double time_increase_;                // 局部轨迹时间重分配后，累计给全局时间轴增加的时间。
    double last_time_inc_;                // 最近一次局部替换带来的时间增量。
    double last_progress_time_;           // FSM 沿全局轨迹推进到的时间，用于下次继续选择局部目标。

    GlobalTrajData(/* args */) {}

    ~GlobalTrajData() {}

    bool localTrajReachTarget() { return fabs(local_end_time_ - global_duration_) < 0.1; }

    void setGlobalTraj(const PolynomialTraj &traj, const ros::Time &time)
    {
      global_traj_ = traj;
      global_traj_.init();
      global_duration_ = global_traj_.getTimeSum();
      global_start_time_ = time;

      local_traj_.clear();
      local_start_time_ = -1;
      local_end_time_ = -1;
      time_increase_ = 0.0;
      last_time_inc_ = 0.0;
      last_progress_time_ = 0.0;
    }

    void setLocalTraj(UniformBspline traj, double local_ts, double local_te, double time_inc)
    {
      local_traj_.resize(3);
      local_traj_[0] = traj;
      local_traj_[1] = local_traj_[0].getDerivative();
      local_traj_[2] = local_traj_[1].getDerivative();

      local_start_time_ = local_ts;
      local_end_time_ = local_te;
      global_duration_ += time_inc;
      time_increase_ += time_inc;
      last_time_inc_ = time_inc;
    }

    Eigen::Vector3d getPosition(double t)
    {
      if (t >= -1e-3 && t <= local_start_time_)
      {
        return global_traj_.evaluate(t - time_increase_ + last_time_inc_);
      }
      else if (t >= local_end_time_ && t <= global_duration_ + 1e-3)
      {
        return global_traj_.evaluate(t - time_increase_);
      }
      else
      {
        double tm, tmp;
        local_traj_[0].getTimeSpan(tm, tmp);
        return local_traj_[0].evaluateDeBoor(tm + t - local_start_time_);
      }
    }

    Eigen::Vector3d getVelocity(double t)
    {
      if (t >= -1e-3 && t <= local_start_time_)
      {
        return global_traj_.evaluateVel(t);
      }
      else if (t >= local_end_time_ && t <= global_duration_ + 1e-3)
      {
        return global_traj_.evaluateVel(t - time_increase_);
      }
      else
      {
        double tm, tmp;
        local_traj_[0].getTimeSpan(tm, tmp);
        return local_traj_[1].evaluateDeBoor(tm + t - local_start_time_);
      }
    }

    Eigen::Vector3d getAcceleration(double t)
    {
      if (t >= -1e-3 && t <= local_start_time_)
      {
        return global_traj_.evaluateAcc(t);
      }
      else if (t >= local_end_time_ && t <= global_duration_ + 1e-3)
      {
        return global_traj_.evaluateAcc(t - time_increase_);
      }
      else
      {
        double tm, tmp;
        local_traj_[0].getTimeSpan(tm, tmp);
        return local_traj_[2].evaluateDeBoor(tm + t - local_start_time_);
      }
    }

    // get Bspline paramterization data of a local trajectory within a sphere
    // start_t: start time of the trajectory
    // dist_pt: distance between the discretized points
    void getTrajByRadius(const double &start_t, const double &des_radius, const double &dist_pt,
                         vector<Eigen::Vector3d> &point_set, vector<Eigen::Vector3d> &start_end_derivative,
                         double &dt, double &seg_duration)
    {
      double seg_length = 0.0; // length of the truncated segment
      double seg_time = 0.0;   // duration of the truncated segment
      double radius = 0.0;     // distance to the first point of the segment

      double delt = 0.2;
      Eigen::Vector3d first_pt = getPosition(start_t); // first point of the segment
      Eigen::Vector3d prev_pt = first_pt;              // previous point
      Eigen::Vector3d cur_pt;                          // current point

      // go forward until the traj exceed radius or global time

      while (radius < des_radius && seg_time < global_duration_ - start_t - 1e-3)
      {
        seg_time += delt;
        seg_time = std::min(seg_time, global_duration_ - start_t);

        cur_pt = getPosition(start_t + seg_time);
        seg_length += (cur_pt - prev_pt).norm();
        prev_pt = cur_pt;
        radius = (cur_pt - first_pt).norm();
      }

      // get parameterization dt by desired density of points
      int seg_num = floor(seg_length / dist_pt);

      // get outputs

      seg_duration = seg_time; // duration of the truncated segment
      dt = seg_time / seg_num; // time difference between two points

      for (double tp = 0.0; tp <= seg_time + 1e-4; tp += dt)
      {
        cur_pt = getPosition(start_t + tp);
        point_set.push_back(cur_pt);
      }

      start_end_derivative.push_back(getVelocity(start_t));
      start_end_derivative.push_back(getVelocity(start_t + seg_time));
      start_end_derivative.push_back(getAcceleration(start_t));
      start_end_derivative.push_back(getAcceleration(start_t + seg_time));
    }

    // get Bspline paramterization data of a fixed duration local trajectory
    // start_t: start time of the trajectory
    // duration: time length of the segment
    // seg_num: discretized the segment into *seg_num* parts
    void getTrajByDuration(double start_t, double duration, int seg_num,
                           vector<Eigen::Vector3d> &point_set,
                           vector<Eigen::Vector3d> &start_end_derivative, double &dt)
    {
      dt = duration / seg_num;
      Eigen::Vector3d cur_pt;
      for (double tp = 0.0; tp <= duration + 1e-4; tp += dt)
      {
        cur_pt = getPosition(start_t + tp);
        point_set.push_back(cur_pt);
      }

      start_end_derivative.push_back(getVelocity(start_t));
      start_end_derivative.push_back(getVelocity(start_t + duration));
      start_end_derivative.push_back(getAcceleration(start_t));
      start_end_derivative.push_back(getAcceleration(start_t + duration));
    }
  };

  /*
   * PlanParameters 保存轨迹规划的核心参数。
   *
   * 这些值在 EGOPlannerManager::initPlanModules() 中从 ROS 参数服务器读取，
   * 对应 launch 文件里的 manager/* 参数。它们决定局部轨迹的速度/加速度约束、
   * B 样条控制点密度、规划 horizon，以及是否尝试多条候选绕障轨迹。
   */
  struct PlanParameters
  {
    /* planning algorithm parameters */
    double max_vel_, max_acc_, max_jerk_; // 轨迹允许的最大速度、最大加速度、最大 jerk。
    double ctrl_pt_dist;                  // 相邻 B 样条控制点期望间距，越小轨迹控制点越密。
    double feasibility_tolerance_;        // 速度/加速度可行性检查的容忍比例。
    double planning_horizen_;             // 局部规划视野长度，FSM 会在这个距离内选局部目标。
    bool use_distinctive_trajs;           // 是否生成多条不同绕障方向的候选轨迹再择优。
    int drone_id;                         // 单机通常 <= -1 或 0；多机模式下用来区分无人机编号。

    /* processing time */
    double time_search_ = 0.0;            // 搜索耗时统计，当前版本中基本没有被维护。
    double time_optimize_ = 0.0;          // 优化耗时统计，当前版本中基本没有被维护。
    double time_adjust_ = 0.0;            // 时间重分配耗时统计，当前版本中基本没有被维护。
  };

  /*
   * LocalTrajData 保存“当前已经接受的局部轨迹”。
   *
   * 每次 reboundReplan() 成功后，EGOPlannerManager 会调用 updateTrajInfo()
   * 更新这个结构体。FSM 用它判断是否需要重规划；轨迹发布逻辑会读取其中的 B 样条，
   * 封装成 traj_utils::Bspline 后发布给 traj_server。
   */
  struct LocalTrajData
  {
    /* info of generated traj */

    int traj_id_;                         // 局部轨迹编号，每成功生成一次就递增。
    double duration_;                     // 当前局部 B 样条轨迹总时长。
    ros::Time start_time_;                // 当前局部轨迹开始执行的 ROS 时间。
    Eigen::Vector3d start_pos_;           // 当前局部轨迹的起点位置。
    UniformBspline position_traj_, velocity_traj_, acceleration_traj_; // 位置轨迹及其一阶、二阶导数。
  };

  // 多机模式下使用的单架无人机轨迹缓存结构；当前单机阅读主线可以先跳过。
  struct OneTrajDataOfSwarm
  {
    /* info of generated traj */

    int drone_id;
    double duration_;
    ros::Time start_time_;
    Eigen::Vector3d start_pos_;
    UniformBspline position_traj_;
  };

  typedef std::vector<OneTrajDataOfSwarm> SwarmTrajData;

} // namespace ego_planner


namespace fastnav
{

  /*
   * GlobalTrajData 保存 FastNav 的全局 MINCO 参考轨迹。
   * 它对应任务级参考线：FSM 可以沿 $p_g(t)$ 选择局部目标，manager 可以用它判断任务是否接近终点。
   * 注意这里保存的是工程运行时数据，底层数学轨迹仍然由 MincoTraj 内部的 Trajectory<5> 表示。
   */
  struct GlobalTrajData
  {
    int traj_id_{0};
    bool valid_{false};
    std::string frame_id_{"odom"};

    ros::Time start_time_{0};
    double duration_{0.0};
    double last_progress_time_{0.0};

    MincoTraj position_traj_;
    std::vector<Eigen::Vector3d> waypoints_;
    std::vector<Eigen::MatrixX4d> corridor_;

    void reset()
    {
      valid_ = false;
      duration_ = 0.0;
      last_progress_time_ = 0.0;
      start_time_ = ros::Time(0);
      position_traj_.clear();
      waypoints_.clear();
      corridor_.clear();
    }

    void setGlobalTraj(const MincoTraj::TrajectoryType& traj,
                       const ros::Time& time,
                       const std::string& frame_id = "odom")
    {
      position_traj_.setTrajectory(traj);
      valid_ = position_traj_.valid();
      duration_ = position_traj_.getDuration();
      start_time_ = time;
      frame_id_ = frame_id;
      last_progress_time_ = 0.0;
      traj_id_ += valid_ ? 1 : 0;
    }

    double elapsedTime(const ros::Time& now) const
    {
      if (!valid_ || start_time_.isZero())
      {
        return 0.0;
      }
      return std::max(0.0, (now - start_time_).toSec());
    }

    bool isExpired(const ros::Time& now) const
    {
      return valid_ && elapsedTime(now) >= duration_ - 1.0e-3;
    }

    Eigen::Vector3d getPosition(double t) const { return position_traj_.getPosition(t); }
    Eigen::Vector3d getVelocity(double t) const { return position_traj_.getVelocity(t); }
    Eigen::Vector3d getAcceleration(double t) const { return position_traj_.getAcceleration(t); }
    Eigen::Vector3d getJerk(double t) const { return position_traj_.getJerk(t); }

    Eigen::Vector3d getPosition(const ros::Time& now) const { return getPosition(elapsedTime(now)); }
    Eigen::Vector3d getVelocity(const ros::Time& now) const { return getVelocity(elapsedTime(now)); }
    Eigen::Vector3d getAcceleration(const ros::Time& now) const { return getAcceleration(elapsedTime(now)); }
  };

  /*
   * LocalTrajData 保存当前已经接受的局部 MINCO 轨迹。
   * planner 每次成功重规划后更新该结构；FSM / controller 可以根据 ROS 时间采样 $p(t),v(t),a(t)$。
   */
  struct LocalTrajData
  {
    int traj_id_{0};
    bool valid_{false};
    std::string frame_id_{"odom"};

    ros::Time start_time_{0};
    double duration_{0.0};
    Eigen::Vector3d start_pos_{Eigen::Vector3d::Zero()};

    MincoTraj position_traj_;
    std::vector<Eigen::Vector3d> sampled_path_;
    std::vector<Eigen::MatrixX4d> corridor_;

    void reset()
    {
      valid_ = false;
      duration_ = 0.0;
      start_time_ = ros::Time(0);
      start_pos_.setZero();
      position_traj_.clear();
      sampled_path_.clear();
      corridor_.clear();
    }

    void setLocalTraj(const MincoTraj::TrajectoryType& traj,
                      const ros::Time& time,
                      const std::string& frame_id = "odom",
                      double sample_dt = 0.05)
    {
      position_traj_.setTrajectory(traj);
      valid_ = position_traj_.valid();
      duration_ = position_traj_.getDuration();
      start_time_ = time;
      frame_id_ = frame_id;
      sampled_path_ = position_traj_.samplePositions(sample_dt);
      start_pos_ = valid_ ? position_traj_.getPosition(0.0) : Eigen::Vector3d::Zero();
      traj_id_ += valid_ ? 1 : 0;
    }

    double elapsedTime(const ros::Time& now) const
    {
      if (!valid_ || start_time_.isZero())
      {
        return 0.0;
      }
      return std::max(0.0, (now - start_time_).toSec());
    }

    bool isExpired(const ros::Time& now) const
    {
      return valid_ && elapsedTime(now) >= duration_ - 1.0e-3;
    }

    Eigen::Vector3d getPosition(double t) const { return position_traj_.getPosition(t); }
    Eigen::Vector3d getVelocity(double t) const { return position_traj_.getVelocity(t); }
    Eigen::Vector3d getAcceleration(double t) const { return position_traj_.getAcceleration(t); }
    Eigen::Vector3d getJerk(double t) const { return position_traj_.getJerk(t); }

    Eigen::Vector3d getPosition(const ros::Time& now) const { return getPosition(elapsedTime(now)); }
    Eigen::Vector3d getVelocity(const ros::Time& now) const { return getVelocity(elapsedTime(now)); }
    Eigen::Vector3d getAcceleration(const ros::Time& now) const { return getAcceleration(elapsedTime(now)); }
    Eigen::Vector3d getJerk(const ros::Time& now) const { return getJerk(elapsedTime(now)); }
  };

} // namespace fastnav

#endif
