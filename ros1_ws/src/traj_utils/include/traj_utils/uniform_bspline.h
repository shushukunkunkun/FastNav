#ifndef _TRAJ_UTILS_UNIFORM_BSPLINE_H_
#define _TRAJ_UTILS_UNIFORM_BSPLINE_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <vector>

namespace ego_planner
{

class UniformBspline
{
private:
  Eigen::MatrixXd control_points_;

  int p_{0}, n_{0}, m_{0};
  Eigen::VectorXd u_;
  double interval_{0.0};

  Eigen::MatrixXd getDerivativeControlPoints();

  double limit_vel_{0.0}, limit_acc_{0.0}, limit_ratio_{1.1}, feasibility_tolerance_{0.0};

public:
  UniformBspline() {}
  UniformBspline(const Eigen::MatrixXd &points, const int &order, const double &interval);
  ~UniformBspline();

  Eigen::MatrixXd get_control_points(void) { return control_points_; }

  void setUniformBspline(const Eigen::MatrixXd &points, const int &order, const double &interval);

  void setKnot(const Eigen::VectorXd &knot);
  Eigen::VectorXd getKnot();
  Eigen::MatrixXd getControlPoint();
  double getInterval();
  bool getTimeSpan(double &um, double &um_p);

  Eigen::VectorXd evaluateDeBoor(const double &u);
  inline Eigen::VectorXd evaluateDeBoorT(const double &t) { return evaluateDeBoor(t + u_(p_)); }
  UniformBspline getDerivative();

  static void parameterizeToBspline(const double &ts, const std::vector<Eigen::Vector3d> &point_set,
                                    const std::vector<Eigen::Vector3d> &start_end_derivative,
                                    Eigen::MatrixXd &ctrl_pts);

  void setPhysicalLimits(const double &vel, const double &acc, const double &tolerance);
  bool checkFeasibility(double &ratio, bool show = false);
  void lengthenTime(const double &ratio);

  double getTimeSum();
  double getLength(const double &res = 0.01);
  double getJerk();
  void getMeanAndMaxVel(double &mean_v, double &max_v);
  void getMeanAndMaxAcc(double &mean_a, double &max_a);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace ego_planner

#endif
