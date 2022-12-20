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

std::pair<double, Eigen::Isometry3d> runGradientDescent(
  const Eigen::Isometry3d & tf_m_to_q, const Eigen::Isometry3d & tf_m_to_e,
  const double theta_start, const std::pair<double, double> theta_limits,
  const ScrewAxis & screw_axis)
{
  // TODO: tune these
  const double gamma = 0.05;
  const size_t max_steps = 100;
  const double converge_limit = 0.001;

  double theta = theta_start;
  size_t i = 0;

  Eigen::Isometry3d tf_q_to_path = tf_m_to_q.inverse() * screw_axis.getTF(theta) * tf_m_to_e;
  Eigen::VectorXd error = calcError(tf_q_to_path);
  double error_norm_diff = std::numeric_limits<double>::max();

  // TODO: use squaredNorm() instead of norm() for faster performance
  while (fabs(error_norm_diff) > converge_limit && i < max_steps) {
    i++;
    const double last_error_norm = error.norm();
    theta -= gamma * calcErrorDerivative(tf_m_to_q, tf_m_to_e, theta, screw_axis);
    error = calcError(tf_m_to_q.inverse() * screw_axis.getTF(theta) * tf_m_to_e);
    error_norm_diff = last_error_norm - error.norm();

    if (theta < theta_limits.first) {
      theta = theta_limits.first;
      break;
    }
    if (theta > theta_limits.second) {
      theta = theta_limits.second;
      break;
    }
  }
  tf_q_to_path = tf_m_to_q.inverse() * screw_axis.getTF(theta) * tf_m_to_e;
  return std::make_pair(theta, tf_q_to_path);
}

std::queue<double> getGradStarts(const std::pair<double, double> & limits, double max_dist)
{
  std::queue<double> output;

  const double span = fabs(limits.second - limits.first);
  const size_t num_starts = ceil(span / max_dist) + 1;
  const double real_step = span / num_starts;

  for (size_t i = 0; i < num_starts; ++i) {
    output.push(limits.first + i * real_step);
  }
  output.push(limits.second);

  return output;
}

bool screwConstraint(
  const Eigen::Isometry3d & current_pose, const Eigen::Isometry3d & start_pose,
  const ScrewAxis & screw_axis, const std::pair<double, double> theta_limits, double theta_guess,
  Eigen::Ref<Eigen::VectorXd> out)
{
  // Find the closest point on the path
  std::pair<double, Eigen::Isometry3d> best_output =
    runGradientDescent(current_pose, start_pose, theta_guess, theta_limits, screw_axis);
  double best_error = calcError(best_output.second).norm();
  auto start_guesses = getGradStarts(theta_limits);
  while (!start_guesses.empty() && best_error > 5e-3) {
    const auto closest_pt =
      runGradientDescent(current_pose, start_pose, start_guesses.front(), theta_limits, screw_axis);
    start_guesses.pop();

    const double current_error = calcError(closest_pt.second).norm();
    if (current_error < best_error) {
      best_output = closest_pt;
      best_error = current_error;
    }
  }

  // Use closest point to calculate error
  auto error = calcError(best_output.second);
  out = error;

  return true;
}
}  // namespace affordance_primitives
