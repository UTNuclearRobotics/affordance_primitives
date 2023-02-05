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
  tf_m_to_s_ = Eigen::Isometry3d::Identity();
  size_ = 0;
}

ScrewConstraint::ScrewConstraint(
  const std::vector<ScrewStamped> & screws, const std::vector<double> & lower_bounds,
  const std::vector<double> & upper_bounds, const Eigen::Isometry3d & tf_m_to_s)
{
  // Make sure input is good
  const size_t m = screws.size();
  if (m < 1 || m != lower_bounds.size() || m != upper_bounds.size()) {
    tf_m_to_s_ = Eigen::Isometry3d::Identity();
    size_ = 0;
  }

  // Add all info
  setReferenceFrame(tf_m_to_s);
  for (size_t i = 0; i < m; ++i) {
    addScrewAxis(screws.at(i), lower_bounds.at(i), upper_bounds.at(i));
  }
}

ScrewConstraint::ScrewConstraint(
  const std::vector<ScrewAxis> & screws, const std::vector<double> & lower_bounds,
  const std::vector<double> & upper_bounds, const Eigen::Isometry3d & tf_m_to_s)
{
  // Make sure input is good
  const size_t m = screws.size();
  if (m < 1 || m != lower_bounds.size() || m != upper_bounds.size()) {
    tf_m_to_s_ = Eigen::Isometry3d::Identity();
    size_ = 0;
  }

  // Add all info
  setReferenceFrame(tf_m_to_s);
  for (size_t i = 0; i < m; ++i) {
    addScrewAxis(screws.at(i), lower_bounds.at(i), upper_bounds.at(i));
  }
}

void ScrewConstraint::setReferenceFrame(const Eigen::Isometry3d & tf_m_to_s)
{
  tf_m_to_s_ = tf_m_to_s;
}

void ScrewConstraint::addScrewAxis(
  const ScrewStamped & axis, double lower_bound, double upper_bound)
{
  ScrewAxis screw_axis;
  screw_axis.setScrewAxis(axis);
  axes_.push_back(screw_axis);

  lower_bounds_.push_back(lower_bound);
  upper_bounds_.push_back(upper_bound);

  size_ = axes_.size();
}

void ScrewConstraint::addScrewAxis(const ScrewAxis & axis, double lower_bound, double upper_bound)
{
  axes_.push_back(axis);

  lower_bounds_.push_back(lower_bound);
  upper_bounds_.push_back(upper_bound);

  size_ = axes_.size();
}
}  // namespace affordance_primitives
