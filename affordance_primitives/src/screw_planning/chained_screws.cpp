#include <affordance_primitives/screw_planning/chained_screws.hpp>

namespace affordance_primitives
{
void getGradStarts(const ChainedScrews * constraint, std::queue<std::vector<double>> & grad_starts)
{
  // Make sure we start twice on each axis
  const double span_limit = 0.99 * M_PI;
  double b_now = 0;
  double b_next = constraint->upperBounds().at(0) - constraint->lowerBounds().at(0);

  // Iterate through axes
  const size_t m = constraint->size();
  for (size_t ax = 0; ax < m; ++ax) {
    const double axis_span = b_next - b_now;
    const size_t num_starts = ceil(axis_span / span_limit);
    if (num_starts == 1) {
      // Start 5% and 95%
      double start_lambda = b_now + 0.05 * axis_span;
      grad_starts.push(constraint->getPhi(start_lambda));
      start_lambda = b_now + 0.95 * axis_span;
      grad_starts.push(constraint->getPhi(start_lambda));
    } else {
      const double spacing = axis_span / (num_starts + 1);
      double start_lambda = b_now;
      for (size_t i = 0; i < num_starts; ++i) {
        start_lambda += spacing;
        grad_starts.push(constraint->getPhi(start_lambda));
      }
    }

    // Update bounds for next axis
    if ((ax + 1) < m) {
      b_now = b_next;
      b_next += constraint->upperBounds().at(ax + 1) - constraint->lowerBounds().at(ax + 1);
    }
  }
}

bool chainedConstraintFn(
  const ChainedScrews * constraint, const Eigen::Isometry3d & tf_m_to_q,
  std::queue<std::vector<double>> & grad_starts, ScrewConstraintSolution & sol)
{
  // Run gradient descent on all the starting positions
  while (!grad_starts.empty()) {
    // Run the descent once
    if (!runGradDescentOnce(constraint, tf_m_to_q, grad_starts.front(), sol)) {
      return false;
    }
    grad_starts.pop();

    // If the solution ever satisfies the constraint, get out early
    if (sol.error < constraint->tolerance()) {
      return true;
    }
  }
  return true;
}

bool runGradDescentOnce(
  const ChainedScrews * constraint, const Eigen::Isometry3d & tf_m_to_q,
  const std::vector<double> & phi0, ScrewConstraintSolution & sol)
{
  // Check the input
  if (phi0.size() != constraint->size()) {
    return false;
  }

  auto phi_now = phi0;

  // Gradient descent parameters
  constexpr double gamma = 0.5;      // learning parameter
  constexpr size_t nmax = 100;       // max steps
  constexpr double epsilon = 0.001;  // converge limit

  // Compute tf_q_to_p
  const Eigen::Isometry3d tf_q_to_m = tf_m_to_q.inverse();
  Eigen::Isometry3d tf_q_to_p = tf_q_to_m * constraint->getPose(phi_now);

  // Carry out the search
  double delta = epsilon;
  size_t i = 0;
  while (i < nmax && fabs(delta) >= epsilon) {
    // Find lambda for this state
    size_t s_index;
    double lambda = constraint->getLambda(phi_now, &s_index);
    double lambda_last = lambda;

    // Calculate the derivative
    auto deriv = computeDerivativeForIndex(constraint, phi_now, s_index, tf_q_to_m, tf_q_to_p);

    // Update lambda
    lambda -= gamma * deriv;

    // Clamp then get phi
    lambda = std::clamp(lambda, constraint->lambdaMin(), constraint->lambdaMax());
    phi_now = constraint->getPhi(lambda);

    // Compute tf_q_to_p
    tf_q_to_p = tf_q_to_m * constraint->getPose(phi_now);

    // Compute new delta
    delta = lambda_last - lambda;
    i++;
  }

  // If we have improved the solution, update it
  const auto error_now = calculateEta(tf_q_to_p);
  if (error_now.norm() < sol.error) {
    sol.solved_phi = phi_now;
    sol.error_vector = error_now;
    sol.error = error_now.norm();
  }
  return true;
}

double computeDerivativeForIndex(
  const ChainedScrews * constraint, const std::vector<double> & phi, size_t index,
  const Eigen::Isometry3d & tf_q_to_m, const Eigen::Isometry3d & tf_q_to_p)
{
  // Calculate the partial derivative of the ``index'' variable
  Eigen::Matrix4d psi = tf_q_to_m.matrix();
  for (size_t i = 0; i < index; ++i) {
    psi = psi * constraint->axes().at(i).getTF(phi[i]).matrix();
  }
  psi = psi * constraint->axes().at(index).getScrewSkewSymmetricMatrix();
  for (size_t i = index; i < constraint->size(); ++i) {
    psi = psi * constraint->axes().at(i).getTF(phi[i]).matrix();
  }
  psi = psi * constraint->referenceFrame().matrix();

  return calculateEta(tf_q_to_p).dot(calculateEta(psi));
}

ChainedScrews::ChainedScrews() : ScrewConstraint(), lambda_max_{0} {};

ChainedScrews::ChainedScrews(
  const std::vector<ScrewStamped> & screws, const std::vector<double> & start_phi,
  const std::vector<double> & goal_phi, const Eigen::Isometry3d & tf_m_to_s,
  const std::vector<double> & lower_bounds, const std::vector<double> & upper_bounds)
: ScrewConstraint(screws, start_phi, goal_phi, tf_m_to_s, lower_bounds, upper_bounds)
{
  lambda_max_ = getLambda(goal_phi);
}

ChainedScrews::ChainedScrews(
  const std::vector<ScrewAxis> & screws, const std::vector<double> & start_phi,
  const std::vector<double> & goal_phi, const Eigen::Isometry3d & tf_m_to_s,
  const std::vector<double> & lower_bounds, const std::vector<double> & upper_bounds)
: ScrewConstraint(screws, start_phi, goal_phi, tf_m_to_s, lower_bounds, upper_bounds)
{
  lambda_max_ = getLambda(goal_phi);
}

void ChainedScrews::addScrewAxis(const ScrewStamped & axis, double start_theta, double end_theta)
{
  ScrewConstraint::addScrewAxis(axis, start_theta, end_theta);
  lambda_max_ = getLambda(goal_phi_);
}

void ChainedScrews::addScrewAxis(
  const ScrewStamped & axis, double start_theta, double end_theta, double lower_bound,
  double upper_bound)
{
  addScrewAxis(axis, start_theta, end_theta);
}

void ChainedScrews::addScrewAxis(const ScrewAxis & axis, double start_theta, double end_theta)
{
  ScrewConstraint::addScrewAxis(axis, start_theta, end_theta);
  lambda_max_ = getLambda(goal_phi_);
}

void ChainedScrews::addScrewAxis(
  const ScrewAxis & axis, double start_theta, double end_theta, double lower_bound,
  double upper_bound)
{
  addScrewAxis(axis, start_theta, end_theta);
}

double ChainedScrews::percentComplete(const std::vector<double> & state) const
{
  // Input checking
  if (state.size() != size_) {
    return 0;
  }

  return getLambda(state) / lambda_max_;
}

std::vector<double> ChainedScrews::sampleUniformState() const
{
  // Draw a sample along the path
  std::uniform_real_distribution<> dis(0, lambda_max_);
  return getPhi(dis(*random_gen_));
}

std::vector<double> ChainedScrews::sampleUniformStateNear(
  const std::vector<double> & near, double distance) const
{
  // Check input
  if (near.size() != size_) {
    return sampleUniformState();
  }

  // Sample near the passed state
  const double lambda = getLambda(near);
  std::uniform_real_distribution<> dis(lambda - 0.5 * distance, lambda + 0.5 * distance);
  const double val = dis(*random_gen_);

  // Enforce bounds
  return getPhi(std::clamp(val, 0.0, lambda_max_));
}

std::vector<double> ChainedScrews::sampleGaussianStateNear(
  const std::vector<double> & mean, double stdDev) const
{
  // Check input
  if (mean.size() != size_) {
    return sampleUniformState();
  }

  // Sample near the passed state (mean, stddev)
  const double lambda = getLambda(mean);
  std::normal_distribution<> dis{lambda, stdDev};
  const double val = dis(*random_gen_);

  // Enforce bounds
  return getPhi(std::clamp(val, 0.0, lambda_max_));
}

Eigen::Isometry3d ChainedScrews::getPose(const std::vector<double> & phi) const
{
  return ScrewConstraint::getPose(phi);
}

Eigen::Isometry3d ChainedScrews::getPose(double lambda) const
{
  // Check input
  if (lambda < lambdaMin() || lambda > lambdaMax()) {
    return Eigen::Isometry3d::Identity();
  }
  return getPose(getPhi(lambda));
}

double ChainedScrews::getLambda(const std::vector<double> & phi, size_t * s_index) const
{
  // Check input
  if (phi.size() != size_) {
    return -1;
  }

  double lambda = 0;
  size_t s = 0;
  const double EPSILON = 1e-6;

  for (s = 0; s < phi.size(); ++s) {
    if (fabs(phi.at(s) - lower_bounds_.at(s)) < EPSILON) {
      if (s_index) {
        *s_index = s == 0 ? 0 : s - 1;
      }
      return lambda;
    } else {
      lambda += phi.at(s) - lower_bounds_.at(s);
    }
  }
  if (s_index) {
    *s_index = s == 0 ? 0 : s - 1;
  }
  return lambda;
}

std::vector<double> ChainedScrews::getPhi(double lambda, size_t * s_index) const
{
  std::vector<double> phi = lower_bounds_;
  size_t s = 0;
  double b_now = 0;
  double b_next = upper_bounds_.at(0) - lower_bounds_.at(0);

  for (size_t i = 0; i < phi.size(); ++i) {
    if (lambda < b_now) {
      phi.at(i) = lower_bounds_.at(i);
    } else if (lambda > b_next) {
      phi.at(i) = upper_bounds_.at(i);
    } else {
      phi.at(i) = lower_bounds_.at(i) + lambda - b_now;
      s = i;
    }

    if ((i + 1) < phi.size()) {
      b_now = b_next;
      b_next += upper_bounds_.at(i + 1) - lower_bounds_.at(i + 1);
    }
  }
  if (s_index) {
    *s_index = s;
  }
  return phi;
}

std::vector<ScrewStamped> ChainedScrews::getVisualScrews() const
{
  std::vector<ScrewStamped> output;
  output.reserve(size_);

  // We will need the start pose of each screw axis, including first
  auto phi = lower_bounds_;
  auto tf_start_of_this_axis = getPose(phi);
  const auto tf_s_to_m = tf_m_to_s_.inverse();

  // Iterate through each axis
  for (size_t i = 0; i < size_; ++i) {
    const auto & screw_i = axes_.at(i);

    // Extract screw info (currently in affordance frame)
    Eigen::Vector3d origin = screw_i.getQVector();
    Eigen::Vector3d axis =
      screw_i.isPureTranslation() ? screw_i.getLinearVector() : screw_i.getAxis();

    // Convert screw to be in frame {S}
    origin = tf_s_to_m.linear() * origin + tf_s_to_m.translation();
    axis = tf_s_to_m.linear() * axis;

    // Find the current start location w.r.t. {S}
    auto tf_s_to_i = tf_s_to_m * tf_start_of_this_axis;

    // Move screw info by that ^ difference
    origin += tf_s_to_i.translation();
    axis = tf_s_to_i.linear() * axis;

    // Put screw info back into frame {M}
    origin = tf_m_to_s_.linear() * origin + tf_m_to_s_.translation();
    axis = tf_m_to_s_.linear() * axis;

    // Put it into message form
    ScrewStamped screw_msg = screw_i.toMsg();
    tf2::toMsg(axis, screw_msg.axis);
    screw_msg.origin = tf2::toMsg(origin);
    output.push_back(screw_msg);

    // Update for next step
    phi.at(i) = upper_bounds_.at(i);
    tf_start_of_this_axis = getPose(phi);
  }
  return output;
}

bool ChainedScrews::constraintFn(const Eigen::Isometry3d & tf_m_to_q, ScrewConstraintSolution & sol)
{
  if (size_ < 1) {
    return false;
  }

  sol.reset();

  // Create a list of gradient descent starts
  std::queue<std::vector<double>> grad_starts;
  getGradStarts(this, grad_starts);

  // Run the actual math
  return chainedConstraintFn(this, tf_m_to_q, grad_starts, sol);
}

bool ChainedScrews::constraintFn(
  const Eigen::Isometry3d & tf_m_to_q, const std::vector<double> & phi_0,
  ScrewConstraintSolution & sol)
{
  if (size_ < 1) {
    return false;
  }

  sol.reset();

  // Make sure the guess goes first in the gradient descent starts
  std::queue<std::vector<double>> grad_starts;
  if (phi_0.size() == size_) {
    grad_starts.push(phi_0);
  }

  // Populate the rest of the starting list
  getGradStarts(this, grad_starts);

  // Run the actual math
  return chainedConstraintFn(this, tf_m_to_q, grad_starts, sol);
}
}  // namespace affordance_primitives
