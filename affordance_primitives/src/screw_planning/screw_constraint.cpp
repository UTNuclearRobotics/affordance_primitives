#include <affordance_primitives/screw_planning/screw_constraint.hpp>

namespace affordance_primitives
{
Eigen::VectorXd calculateEta(const Eigen::Matrix4d & tf)
{
  const Eigen::AngleAxisd axisAngleRep(tf.block<3, 3>(0, 0));
  const Eigen::Vector3d lin = tf.block<3, 1>(0, 3);

  Eigen::VectorXd eta(6);
  eta.head(3) = lin;
  eta.tail(3) = axisAngleRep.angle() * axisAngleRep.axis();
  return eta;
}

Eigen::VectorXd calculateEta(const Eigen::Isometry3d & tf) { return calculateEta(tf.matrix()); }

ScrewConstraint::ScrewConstraint()
{
  random_gen_ = std::make_unique<std::mt19937>((std::random_device())());
  tf_m_to_s_ = Eigen::Isometry3d::Identity();
  size_ = 0;
}

ScrewConstraint::ScrewConstraint(
  const std::vector<ScrewStamped> & screws, const std::vector<double> & start_phi,
  const std::vector<double> & goal_phi, const Eigen::Isometry3d & tf_m_to_s,
  const std::vector<double> & lower_bounds, const std::vector<double> & upper_bounds)
{
  random_gen_ = std::make_unique<std::mt19937>((std::random_device())());
  // Make sure input is good
  const size_t m = screws.size();
  if (m < 1 || m != start_phi.size() || m != goal_phi.size()) {
    tf_m_to_s_ = Eigen::Isometry3d::Identity();
    size_ = 0;
  }

  // Add all info
  setReferenceFrame(tf_m_to_s);
  for (size_t i = 0; i < m; ++i) {
    addScrewAxis(screws.at(i), start_phi.at(i), upper_bounds.at(i));
  }

  // Check and set bounds accordingly
  if (m == lower_bounds.size() && m == upper_bounds.size()) {
    lower_bounds_ = lower_bounds;
    upper_bounds_ = upper_bounds;
  } else {
    lower_bounds_.reserve(m);
    upper_bounds_.reserve(m);
    for (size_t i = 0; i < m; ++i) {
      lower_bounds_.push_back(std::min(start_phi.at(i), goal_phi.at(i)));
      upper_bounds_.push_back(std::max(start_phi.at(i), goal_phi.at(i)));
    }
  }
}

ScrewConstraint::ScrewConstraint(
  const std::vector<ScrewAxis> & screws, const std::vector<double> & start_phi,
  const std::vector<double> & goal_phi, const Eigen::Isometry3d & tf_m_to_s,
  const std::vector<double> & lower_bounds, const std::vector<double> & upper_bounds)
{
  random_gen_ = std::make_unique<std::mt19937>((std::random_device())());
  // Make sure input is good
  const size_t m = screws.size();
  if (m < 1 || m != start_phi.size() || m != goal_phi.size()) {
    tf_m_to_s_ = Eigen::Isometry3d::Identity();
    size_ = 0;
  }

  // Add all info
  setReferenceFrame(tf_m_to_s);
  for (size_t i = 0; i < m; ++i) {
    addScrewAxis(screws.at(i), start_phi.at(i), goal_phi.at(i));
  }

  // Check and set bounds accordingly
  if (m == lower_bounds.size() && m == upper_bounds.size()) {
    lower_bounds_ = lower_bounds;
    upper_bounds_ = upper_bounds;
  } else {
    lower_bounds_.reserve(m);
    upper_bounds_.reserve(m);
    for (size_t i = 0; i < m; ++i) {
      lower_bounds_.push_back(std::min(start_phi.at(i), goal_phi.at(i)));
      upper_bounds_.push_back(std::max(start_phi.at(i), goal_phi.at(i)));
    }
  }
}

Eigen::Isometry3d ScrewConstraint::getPose(const std::vector<double> & phi) const
{
  Eigen::Isometry3d output = Eigen::Isometry3d::Identity();

  // Check input
  if (phi.size() != size_) {
    return output;
  }

  // Step through each axis and calculate
  for (size_t i = 0; i < size_; ++i) {
    output = output * axes_.at(i).getTF(phi[i]);
  }
  output = output * tf_m_to_s_;
  return output;
}

void ScrewConstraint::setReferenceFrame(const Eigen::Isometry3d & tf_m_to_s)
{
  tf_m_to_s_ = tf_m_to_s;
}

void ScrewConstraint::addScrewAxis(const ScrewStamped & axis, double start_theta, double end_theta)
{
  ScrewAxis screw_axis;
  screw_axis.setScrewAxis(axis);
  axes_.push_back(screw_axis);

  start_phi_.push_back(start_theta);
  goal_phi_.push_back(end_theta);
  lower_bounds_.push_back(std::min(start_theta, end_theta));
  upper_bounds_.push_back(std::max(start_theta, end_theta));

  size_ = axes_.size();
}

void ScrewConstraint::addScrewAxis(
  const ScrewStamped & axis, double start_theta, double end_theta, double lower_bound,
  double upper_bound)
{
  ScrewAxis screw_axis;
  screw_axis.setScrewAxis(axis);
  axes_.push_back(screw_axis);

  start_phi_.push_back(start_theta);
  goal_phi_.push_back(end_theta);
  lower_bounds_.push_back(lower_bound);
  upper_bounds_.push_back(upper_bound);

  size_ = axes_.size();
}

void ScrewConstraint::addScrewAxis(const ScrewAxis & axis, double start_theta, double end_theta)
{
  axes_.push_back(axis);

  start_phi_.push_back(start_theta);
  goal_phi_.push_back(end_theta);
  lower_bounds_.push_back(std::min(start_theta, end_theta));
  upper_bounds_.push_back(std::max(start_theta, end_theta));

  size_ = axes_.size();
}

void ScrewConstraint::addScrewAxis(
  const ScrewAxis & axis, double start_theta, double end_theta, double lower_bound,
  double upper_bound)
{
  axes_.push_back(axis);

  start_phi_.push_back(start_theta);
  goal_phi_.push_back(end_theta);
  lower_bounds_.push_back(lower_bound);
  upper_bounds_.push_back(upper_bound);

  size_ = axes_.size();
}

std::vector<double> ScrewConstraint::sampleUniformState() const
{
  std::vector<double> output;
  output.reserve(size_);

  // Iterate through the axes
  for (size_t i = 0; i < size_; ++i) {
    // Generate random numbers within the bounds
    std::uniform_real_distribution<> dis(lower_bounds_[i], upper_bounds_[i]);
    output.push_back(dis(*random_gen_));
  }

  return output;
}

std::vector<double> ScrewConstraint::sampleUniformStateNear(
  const std::vector<double> & near, double distance) const
{
  // Check input
  if (near.size() != size_) {
    return sampleUniformState();
  }

  std::vector<double> output;
  output.reserve(size_);

  // Iterate through the axes
  for (size_t i = 0; i < size_; ++i) {
    // Generate random numbers within the distance
    std::uniform_real_distribution<> dis(near[i] - 0.5 * distance, near[i] + 0.5 * distance);
    const double val = dis(*random_gen_);

    // Make sure its in the bounds
    output.push_back(std::clamp(val, lower_bounds_[i], upper_bounds_[i]));
  }

  return output;
}

std::vector<double> ScrewConstraint::sampleGaussianStateNear(
  const std::vector<double> & mean, double stdDev) const
{
  // Check input
  if (mean.size() != size_) {
    return sampleUniformState();
  }

  std::vector<double> output;
  output.reserve(size_);

  // Iterate through the axes
  for (size_t i = 0; i < size_; ++i) {
    // Generate random numbers (mean, stddev)
    std::normal_distribution<> dis{mean[i], stdDev};
    const double val = dis(*random_gen_);

    // Make sure its in the bounds
    output.push_back(std::clamp(val, lower_bounds_[i], upper_bounds_[i]));
  }

  return output;
}

std::vector<ScrewStamped> ScrewConstraint::getVisualScrews() const
{
  std::vector<ScrewStamped> output;

  for (const auto & axis : axes_) {
    output.push_back(axis.toMsg());
  }

  return output;
}
}  // namespace affordance_primitives
