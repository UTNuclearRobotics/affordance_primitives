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
  const std::vector<Eigen::Isometry3d> & tf_m_to_q, const std::vector<Eigen::Isometry3d> & tf_m_to_s,
  const std::vector<double> phi_start, const std::vector<double> phi_max, const std::vector<ScrewAxis> & phi)
{
  // TODO: tune these
  const std::vector<double> gamma(phi.size(), 0.05);
  const size_t max_steps = 100;
  const double converge_limit = 0.001;

  std::vector<double> phi = phi_start;//redef of phi - to be reworked
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

bool constraintFnMulti(
  const std::vector<Eigen::Isometry3d> & tf_m_to_q, const std::vector<Eigen::Isometry3d> & tf_m_to_s,
  const std::vector<ScrewAxis> & phi, std::vector<double> phi_max, std::vector<double> phi_guess,
  Eigen::Ref<Eigen::VectorXd> phi_out)//Data type of phi_out?
{
  // Find the closest point on the path
  const auto closest_pt =
    findClosestPoint(tf_m_to_q, tf_m_to_s, phi_guess, phi_max, phi);

  // Use closest point to calculate error
  auto error = calcError(closest_pt.second);
  phi_out = error;

  return true;
}
}  // namespace affordance_primitives
