#include <traj_utils/uniform_bspline.h>

#include <cmath>

namespace ego_planner
{

UniformBspline::UniformBspline(const Eigen::MatrixXd &points, const int &order, const double &interval)
{
  setUniformBspline(points, order, interval);
}

UniformBspline::~UniformBspline() {}

void UniformBspline::setUniformBspline(const Eigen::MatrixXd &points, const int &order, const double &interval)
{
  control_points_ = points;
  p_ = order;
  interval_ = interval;

  n_ = points.cols() - 1;
  m_ = n_ + p_ + 1;

  u_ = Eigen::VectorXd::Zero(m_ + 1);
  for (int i = 0; i <= m_; ++i)
  {
    if (i <= p_)
      u_(i) = double(-p_ + i) * interval_;
    else
      u_(i) = u_(i - 1) + interval_;
  }
}

void UniformBspline::setKnot(const Eigen::VectorXd &knot) { u_ = knot; }

Eigen::VectorXd UniformBspline::getKnot() { return u_; }

Eigen::MatrixXd UniformBspline::getControlPoint() { return control_points_; }

double UniformBspline::getInterval() { return interval_; }

bool UniformBspline::getTimeSpan(double &um, double &um_p)
{
  if (p_ > u_.rows() || m_ - p_ > u_.rows())
    return false;

  um = u_(p_);
  um_p = u_(m_ - p_);
  return true;
}

Eigen::VectorXd UniformBspline::evaluateDeBoor(const double &u)
{
  if (control_points_.cols() == 0 || u_.rows() == 0)
    return Eigen::VectorXd::Zero(control_points_.rows());

  double ub = std::min(std::max(u_(p_), u), u_(m_ - p_));

  int k = p_;
  while (k + 1 < u_.rows())
  {
    if (u_(k + 1) >= ub)
      break;
    ++k;
  }

  std::vector<Eigen::VectorXd> d;
  d.reserve(p_ + 1);
  for (int i = 0; i <= p_; ++i)
    d.push_back(control_points_.col(k - p_ + i));

  for (int r = 1; r <= p_; ++r)
  {
    for (int i = p_; i >= r; --i)
    {
      const double denominator = u_[i + 1 + k - r] - u_[i + k - p_];
      const double alpha = std::abs(denominator) > 1e-9 ? (ub - u_[i + k - p_]) / denominator : 0.0;
      d[i] = (1.0 - alpha) * d[i - 1] + alpha * d[i];
    }
  }

  return d[p_];
}

Eigen::MatrixXd UniformBspline::getDerivativeControlPoints()
{
  Eigen::MatrixXd ctp(control_points_.rows(), std::max(0, int(control_points_.cols()) - 1));
  for (int i = 0; i < ctp.cols(); ++i)
  {
    const double denominator = u_(i + p_ + 1) - u_(i + 1);
    if (std::abs(denominator) > 1e-9)
      ctp.col(i) = p_ * (control_points_.col(i + 1) - control_points_.col(i)) / denominator;
    else
      ctp.col(i).setZero();
  }
  return ctp;
}

UniformBspline UniformBspline::getDerivative()
{
  Eigen::MatrixXd ctp = getDerivativeControlPoints();
  UniformBspline derivative(ctp, p_ - 1, interval_);

  if (u_.rows() >= 2)
  {
    Eigen::VectorXd knot(u_.rows() - 2);
    knot = u_.segment(1, u_.rows() - 2);
    derivative.setKnot(knot);
  }

  return derivative;
}

void UniformBspline::parameterizeToBspline(const double &ts,
                                           const std::vector<Eigen::Vector3d> &point_set,
                                           const std::vector<Eigen::Vector3d> &start_end_derivative,
                                           Eigen::MatrixXd &ctrl_pts)
{
  if (ts <= 0 || point_set.size() <= 3)
  {
    std::cout << "[B-spline]: invalid point set or time step." << std::endl;
    return;
  }

  if (start_end_derivative.size() != 4)
  {
    std::cout << "[B-spline]: derivatives error." << std::endl;
    return;
  }

  const int K = point_set.size();
  Eigen::Vector3d prow(3), vrow(3), arow(3);
  prow << 1, 4, 1;
  vrow << -1, 0, 1;
  arow << 1, -2, 1;

  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(K + 4, K + 2);
  for (int i = 0; i < K; ++i)
    A.block(i, i, 1, 3) = (1.0 / 6.0) * prow.transpose();

  A.block(K, 0, 1, 3) = (1.0 / 2.0 / ts) * vrow.transpose();
  A.block(K + 1, K - 1, 1, 3) = (1.0 / 2.0 / ts) * vrow.transpose();
  A.block(K + 2, 0, 1, 3) = (1.0 / ts / ts) * arow.transpose();
  A.block(K + 3, K - 1, 1, 3) = (1.0 / ts / ts) * arow.transpose();

  Eigen::VectorXd bx(K + 4), by(K + 4), bz(K + 4);
  for (int i = 0; i < K; ++i)
  {
    bx(i) = point_set[i](0);
    by(i) = point_set[i](1);
    bz(i) = point_set[i](2);
  }

  for (int i = 0; i < 4; ++i)
  {
    bx(K + i) = start_end_derivative[i](0);
    by(K + i) = start_end_derivative[i](1);
    bz(K + i) = start_end_derivative[i](2);
  }

  Eigen::VectorXd px = A.colPivHouseholderQr().solve(bx);
  Eigen::VectorXd py = A.colPivHouseholderQr().solve(by);
  Eigen::VectorXd pz = A.colPivHouseholderQr().solve(bz);

  ctrl_pts.resize(3, K + 2);
  ctrl_pts.row(0) = px.transpose();
  ctrl_pts.row(1) = py.transpose();
  ctrl_pts.row(2) = pz.transpose();
}

void UniformBspline::setPhysicalLimits(const double &vel, const double &acc, const double &tolerance)
{
  limit_vel_ = vel;
  limit_acc_ = acc;
  limit_ratio_ = 1.1;
  feasibility_tolerance_ = tolerance;
}

bool UniformBspline::checkFeasibility(double &ratio, bool show)
{
  bool feasible = true;
  Eigen::MatrixXd P = control_points_;
  const int dimension = control_points_.rows();

  double max_vel = -1.0;
  const double enlarged_vel_lim = limit_vel_ * (1.0 + feasibility_tolerance_) + 1e-4;
  for (int i = 0; i < P.cols() - 1; ++i)
  {
    Eigen::VectorXd vel = p_ * (P.col(i + 1) - P.col(i)) / (u_(i + p_ + 1) - u_(i + 1));
    if (std::fabs(vel(0)) > enlarged_vel_lim ||
        std::fabs(vel(1)) > enlarged_vel_lim ||
        std::fabs(vel(2)) > enlarged_vel_lim)
    {
      if (show)
        std::cout << "[Check]: Infeasible vel " << i << " :" << vel.transpose() << std::endl;
      feasible = false;
      for (int j = 0; j < dimension; ++j)
        max_vel = std::max(max_vel, std::fabs(vel(j)));
    }
  }

  double max_acc = -1.0;
  const double enlarged_acc_lim = limit_acc_ * (1.0 + feasibility_tolerance_) + 1e-4;
  for (int i = 0; i < P.cols() - 2; ++i)
  {
    Eigen::VectorXd acc = p_ * (p_ - 1) *
                          ((P.col(i + 2) - P.col(i + 1)) / (u_(i + p_ + 2) - u_(i + 2)) -
                           (P.col(i + 1) - P.col(i)) / (u_(i + p_ + 1) - u_(i + 1))) /
                          (u_(i + p_ + 1) - u_(i + 2));

    if (std::fabs(acc(0)) > enlarged_acc_lim ||
        std::fabs(acc(1)) > enlarged_acc_lim ||
        std::fabs(acc(2)) > enlarged_acc_lim)
    {
      if (show)
        std::cout << "[Check]: Infeasible acc " << i << " :" << acc.transpose() << std::endl;
      feasible = false;
      for (int j = 0; j < dimension; ++j)
        max_acc = std::max(max_acc, std::fabs(acc(j)));
    }
  }

  ratio = std::max(max_vel / limit_vel_, std::sqrt(std::fabs(max_acc) / limit_acc_));
  return feasible;
}

void UniformBspline::lengthenTime(const double &ratio)
{
  int num1 = 5;
  int num2 = getKnot().rows() - 1 - 5;

  double delta_t = (ratio - 1.0) * (u_(num2) - u_(num1));
  double t_inc = delta_t / double(num2 - num1);
  for (int i = num1 + 1; i <= num2; ++i)
    u_(i) += double(i - num1) * t_inc;
  for (int i = num2 + 1; i < u_.rows(); ++i)
    u_(i) += delta_t;
}

double UniformBspline::getTimeSum()
{
  double tm, tmp;
  if (getTimeSpan(tm, tmp))
    return tmp - tm;
  return -1.0;
}

double UniformBspline::getLength(const double &res)
{
  double length = 0.0;
  double dur = getTimeSum();
  Eigen::VectorXd p_l = evaluateDeBoorT(0.0), p_n;
  for (double t = res; t <= dur + 1e-4; t += res)
  {
    p_n = evaluateDeBoorT(t);
    length += (p_n - p_l).norm();
    p_l = p_n;
  }
  return length;
}

double UniformBspline::getJerk()
{
  UniformBspline jerk_traj = getDerivative().getDerivative().getDerivative();
  Eigen::VectorXd times = jerk_traj.getKnot();
  Eigen::MatrixXd ctrl_pts = jerk_traj.getControlPoint();
  int dimension = ctrl_pts.rows();

  double jerk = 0.0;
  for (int i = 0; i < ctrl_pts.cols(); ++i)
    for (int j = 0; j < dimension; ++j)
      jerk += (times(i + 1) - times(i)) * ctrl_pts(j, i) * ctrl_pts(j, i);
  return jerk;
}

void UniformBspline::getMeanAndMaxVel(double &mean_v, double &max_v)
{
  UniformBspline vel = getDerivative();
  double tm, tmp;
  vel.getTimeSpan(tm, tmp);

  max_v = -1.0;
  mean_v = 0.0;
  int num = 0;
  for (double t = tm; t <= tmp; t += 0.01)
  {
    const double vn = vel.evaluateDeBoor(t).norm();
    mean_v += vn;
    ++num;
    max_v = std::max(max_v, vn);
  }
  if (num > 0)
    mean_v /= double(num);
}

void UniformBspline::getMeanAndMaxAcc(double &mean_a, double &max_a)
{
  UniformBspline acc = getDerivative().getDerivative();
  double tm, tmp;
  acc.getTimeSpan(tm, tmp);

  max_a = -1.0;
  mean_a = 0.0;
  int num = 0;
  for (double t = tm; t <= tmp; t += 0.01)
  {
    const double an = acc.evaluateDeBoor(t).norm();
    mean_a += an;
    ++num;
    max_a = std::max(max_a, an);
  }
  if (num > 0)
    mean_a /= double(num);
}

}  // namespace ego_planner
