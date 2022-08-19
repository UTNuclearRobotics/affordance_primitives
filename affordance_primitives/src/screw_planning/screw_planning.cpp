#include <affordance_primitives/screw_planning/screw_planning.hpp>

namespace affordance_primitives
{
bool decodeConstraintsMsg(
  const moveit_msgs::Constraints & msg, ScrewAxis & screw_axis, Eigen::Isometry3d & start_pose,
  std::string & link_name)
{
  // Check for bad input
  if (
    msg.position_constraints.size() < 1 ||
    msg.position_constraints.at(0).constraint_region.primitive_poses.size() < 2 ||
    msg.position_constraints.at(0).constraint_region.primitives.size() < 1) {
    return false;
  }

  // The link name is easy
  link_name = msg.position_constraints.at(0).link_name;

  // The start pose is also easy, just need to convert types
  // Note: this is the SECOND pose in the list
  auto start_pose_msg = msg.position_constraints.at(0).constraint_region.primitive_poses.at(1);
  Eigen::Isometry3d starting_pose;
  tf2::fromMsg(start_pose_msg, starting_pose);
  start_pose = starting_pose;

  // Extract the screw axis origin
  // Note: It comes in defined in the planning frame
  const auto constraint_pose =
    msg.position_constraints.at(0).constraint_region.primitive_poses.at(0);
  affordance_primitives::ScrewStamped screw_in_planning;
  screw_in_planning.origin = constraint_pose.position;

  // The axis is the pose's x-axis
  Eigen::Quaterniond constraint_quat;
  tf2::fromMsg(constraint_pose.orientation, constraint_quat);
  const Eigen::Matrix3d constraint_rot_mat(constraint_quat);
  tf2::toMsg(constraint_rot_mat.col(0), screw_in_planning.axis);

  // Extract the pitch information
  // Use box dimensions. First value == is_pure_translation. Second value == pitch
  screw_in_planning.is_pure_translation =
    msg.position_constraints.at(0).constraint_region.primitives.at(0).dimensions.at(0) == 1.0;
  screw_in_planning.pitch =
    msg.position_constraints.at(0).constraint_region.primitives.at(0).dimensions.at(1);

  // Now transform this screw message into the "starting pose"
  const auto screw_in_starting = transformScrew(screw_in_planning, starting_pose);

  // Set the screw axis
  screw_axis.setScrewAxis(screw_in_starting);

  return true;
}

bool encodeConstraintsMsg(
  const ScrewAxis & screw_axis, const Eigen::Isometry3d & start_pose, const std::string & link_name,
  moveit_msgs::Constraints & msg)
{
  // Almost all the information will be in the position constraint
  moveit_msgs::PositionConstraint pos_constraint;
  pos_constraint.link_name = link_name;
  pos_constraint.weight = 1.0;

  // Use the box primitive to set pitch
  const double first_val = screw_axis.isPureTranslation() ? 1.0 : 0.01;
  shape_msgs::SolidPrimitive solid_primitive;
  solid_primitive.type = shape_msgs::SolidPrimitive::BOX;
  solid_primitive.dimensions = {first_val, screw_axis.getPitch(), 1.0};
  pos_constraint.constraint_region.primitives.emplace_back(solid_primitive);

  // Figure out the screw's pose: x-axis pointing down the axis, origin is same
  geometry_msgs::Pose screw_pose;
  screw_pose.position = tf2::toMsg(screw_axis.getQVector());

  // Set the x-axis
  const auto x_axis = screw_axis.getAxis().normalized();

  // Y-axis is orthonormal to x-axis, found using Gram-Schimdt process
  Eigen::Vector3d y_axis(0.25, 0.25, 0.5);
  y_axis = (y_axis - (x_axis.dot(y_axis) / x_axis.dot(x_axis)) * x_axis).normalized();

  // Z-axis is the cross product of the other two
  Eigen::Vector3d z_axis = (x_axis.cross(y_axis)).normalized();

  // Stuff into a rotation matrix and convert to quaternion
  Eigen::Matrix3d rot_mat;
  rot_mat.col(0) = x_axis;
  rot_mat.col(1) = y_axis;
  rot_mat.col(2) = z_axis;
  Eigen::Quaterniond rot_quat(rot_mat);
  screw_pose.orientation = tf2::toMsg(rot_quat);

  // Stuff screw pose into constraints message
  pos_constraint.constraint_region.primitive_poses.emplace_back(screw_pose);

  // Now put the starting pose in
  geometry_msgs::Pose start_pose_msg = tf2::toMsg(start_pose);
  pos_constraint.constraint_region.primitive_poses.emplace_back(start_pose_msg);

  // Put the position constraint in and set the message name
  msg.position_constraints.emplace_back(pos_constraint);
  msg.name = "screw_constraint";

  return true;
}

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
  const Eigen::Isometry3d & tf_current_to_start, const double current_theta,
  const ScrewAxis & screw_axis)
{
  const Eigen::Isometry3d tf_q_to_path = tf_current_to_start * screw_axis.getTF(current_theta);
  const Eigen::Vector3d v = current_theta * screw_axis.getLinearVector();
  const Eigen::Vector3d w = current_theta * screw_axis.getAxis();
  const Eigen::AngleAxisd angle_error(
    tf_q_to_path.linear() * affordance_primitives::getSkewSymmetricMatrix(w));

  Eigen::VectorXd d_error(6);
  d_error.head(3) = tf_q_to_path.linear() * v;
  d_error.tail(3) = angle_error.angle() * angle_error.axis();

  return calcError(tf_q_to_path).dot(d_error);
}

std::pair<double, Eigen::Isometry3d> findClosestPoint(
  const Eigen::Isometry3d & tf_q_to_e, const double theta_start, const double theta_max,
  const ScrewAxis & screw_axis)
{
  // TODO: tune these
  const double gamma = 0.05;
  const size_t max_steps = 100;
  const double converge_limit = 0.001;

  double theta = theta_start;
  size_t i = 0;

  Eigen::Isometry3d tf_q_to_path = tf_q_to_e * screw_axis.getTF(theta);
  Eigen::VectorXd error = calcError(tf_q_to_path);
  double error_norm_diff = 2 * converge_limit;

  // TODO: use squaredNorm() instead of norm() for faster performance
  while (fabs(error_norm_diff) > converge_limit && i < max_steps) {
    i++;
    const double last_error_norm = error.norm();
    theta -= gamma * calcErrorDerivative(tf_q_to_e, theta, screw_axis);
    error = calcError(tf_q_to_e * screw_axis.getTF(theta));
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
  tf_q_to_path = tf_q_to_e * screw_axis.getTF(theta);
  return std::make_pair(theta, tf_q_to_path);
}

bool constraintFn(
  const Eigen::Isometry3d & current_pose, const Eigen::Isometry3d & start_pose,
  const ScrewAxis & screw_axis, double theta_max, double theta_guess,
  Eigen::Ref<Eigen::VectorXd> out)
{
  // Find TF q -> starting
  const auto tf_q_to_starting = current_pose.inverse() * start_pose;

  // Find the closest point on the path
  const auto closest_pt = findClosestPoint(tf_q_to_starting, theta_guess, theta_max, screw_axis);

  // Use closest point to calculate error
  auto error = calcError(closest_pt.second);
  out = error;

  return true;
}
}  // namespace affordance_primitives
