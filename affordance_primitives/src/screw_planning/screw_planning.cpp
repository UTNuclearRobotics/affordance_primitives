#include <affordance_primitives/screw_planning/screw_planning.hpp>

namespace affordance_primitives
{
Eigen::VectorXd calcError(const Eigen::Isometry3d & tf_err)
{
  Eigen::VectorXd error(6);
  error.setZero();

  const Eigen::AngleAxisd angle_err(tf_err.linear());
  error.head(3) = tf_err.translation();
  error.tail(3) = angle_err.angle() * angle_err.axis();

  return error;
}

double calcErrorDerivative(
  const Eigen::Isometry3d & tf_m_to_q, const Eigen::Isometry3d & tf_m_to_e,
  const double current_theta, const ScrewAxis & screw_axis)
{
  const Eigen::Isometry3d tf_q_to_path =
    tf_m_to_q.inverse() * screw_axis.getTF(current_theta) * tf_m_to_e;
  const Eigen::Vector3d v = screw_axis.getLinearVector();
  const Eigen::Vector3d w = screw_axis.getAxis();
  const Eigen::AngleAxisd angle_error(
    tf_q_to_path.linear() * affordance_primitives::getSkewSymmetricMatrix(w));

  Eigen::VectorXd d_error(6);
  d_error.head(3) = tf_q_to_path.linear() * v;
  d_error.tail(3) = angle_error.angle() * angle_error.axis();

  return calcError(tf_q_to_path).dot(d_error);
}

std::pair<double, Eigen::Isometry3d> findClosestPoint(
  const Eigen::Isometry3d & tf_m_to_q, const Eigen::Isometry3d & tf_m_to_e,
  const double theta_start, const double theta_max, const ScrewAxis & screw_axis)
{
  // TODO: tune these
  const double gamma = 0.05;
  const size_t max_steps = 100;
  const double converge_limit = 0.001;

  double theta = theta_start;
  size_t i = 0;

  Eigen::Isometry3d tf_q_to_path = tf_m_to_q.inverse() * screw_axis.getTF(theta) * tf_m_to_e;
  Eigen::VectorXd error = calcError(tf_q_to_path);
  double error_norm_diff = 2 * converge_limit;

  // TODO: use squaredNorm() instead of norm() for faster performance
  while (fabs(error_norm_diff) > converge_limit && i < max_steps) {
    i++;
    const double last_error_norm = error.norm();
    theta -= gamma * calcErrorDerivative(tf_m_to_q, tf_m_to_e, theta, screw_axis);
    error = calcError(tf_m_to_q.inverse() * screw_axis.getTF(theta) * tf_m_to_e);
    error_norm_diff = last_error_norm - error.norm();

    if (theta < 0) {
      theta = 0;
      break;
    }
    if (theta > theta_max) {
      theta = theta_max;
      break;
    }
  }
  tf_q_to_path = tf_m_to_q.inverse() * screw_axis.getTF(theta) * tf_m_to_e;
  return std::make_pair(theta, tf_q_to_path);
}

bool constraintFn(
  const Eigen::Isometry3d & current_pose, const Eigen::Isometry3d & start_pose,
  const ScrewAxis & screw_axis, double theta_max, double theta_guess,
  Eigen::Ref<Eigen::VectorXd> out)
{
  // Find the closest point on the path
  const auto closest_pt =
    findClosestPoint(current_pose, start_pose, theta_guess, theta_max, screw_axis);

  // Use closest point to calculate error
  auto error = calcError(closest_pt.second);
  out = error;

  return true;
}
}  // namespace affordance_primitives
