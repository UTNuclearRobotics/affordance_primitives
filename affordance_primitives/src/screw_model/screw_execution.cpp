#include <affordance_primitives/screw_model/affordance_utils.hpp>
#include <affordance_primitives/screw_model/screw_axis.hpp>
#include <affordance_primitives/screw_model/screw_execution.hpp>

namespace affordance_primitives
{
Eigen::Matrix<double, 6, 1> calculateAffordanceTwist(const ScrewStamped & screw, double theta_dot)
{
  // ScrewAxis does most of the heavy lifting
  ScrewAxis screw_axis;
  screw_axis.setScrewAxis(screw);
  TwistStamped affordance_twist_msg = screw_axis.getTwist(theta_dot);

  // Convert to Eigen Type and return
  Eigen::Matrix<double, 6, 1> affordance_twist;
  tf2::fromMsg(affordance_twist_msg.twist, affordance_twist);
  return affordance_twist;
}

Eigen::Matrix<double, 6, 1> calculateAffordanceWrench(
  const ScrewStamped & screw, const Eigen::Matrix<double, 6, 1> & twist,
  double impedance_translation, double impedance_rotation)
{
  Eigen::Matrix<double, 6, 1> affordance_wrench;
  if (screw.is_pure_translation) {
    // Pure Translation: No torque, force is just along direction of motion
    affordance_wrench.head(3) = impedance_translation * twist.head(3);
    affordance_wrench.tail(3).setZero();
  } else {
    // Rotation cases: torque comes from angular velocity, force from the linear velocity
    affordance_wrench.head(3) = impedance_translation * screw.pitch * twist.tail(3);
    affordance_wrench.tail(3) = impedance_rotation * twist.tail(3);
  }
  return affordance_wrench;
}

Eigen::Matrix<double, 6, 1> calculateAppliedWrench(
  const Eigen::Matrix<double, 6, 1> & affordance_wrench,
  const Eigen::Isometry3d & tf_moving_to_task, const ScrewStamped & screw)
{
  Eigen::Matrix<double, 6, 1> wrench_to_apply;

  // Convert the affordance_wrench to the moving frame
  Eigen::Matrix<double, 6, 1> moving_wrench = transformWrench(affordance_wrench, tf_moving_to_task);

  // Convert screw origin to Eigen types
  Eigen::Vector3d screw_origin;
  tf2::fromMsg(screw.origin, screw_origin);

  // Calculate wrench to apply
  wrench_to_apply.tail(3) = moving_wrench.tail(3);
  Eigen::Vector3d radius =
    tf_moving_to_task.translation() + tf_moving_to_task.linear() * screw_origin;
  wrench_to_apply.head(3) =
    moving_wrench.head(3) + radius.cross(Eigen::Vector3d(moving_wrench.tail(3)));
  return wrench_to_apply;
}

APScrewExecutor::APScrewExecutor(rclcpp::Node::SharedPtr node) : node_(node)
{
  tfBuffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  tfListener_ = std::make_unique<tf2_ros::TransformListener>(*tfBuffer_);
}

bool APScrewExecutor::setStreamingCommands(
  const AffordancePrimitiveGoal & req, AffordancePrimitiveFeedback & feedback)
{
  // Lookup or check passed TF info
  std::optional<TransformStamped> tfmsg_moving_to_task = getTFInfo(req);
  if (!tfmsg_moving_to_task.has_value()) {
    return false;
  }
  Eigen::Isometry3d tf_moving_to_task = tf2::transformToEigen(*tfmsg_moving_to_task);

  // Set the sign of the velocity cmd to be same as requested delta
  const double theta_dot = copysign(req.theta_dot, req.screw_distance);

  // Calculate the twist and wrench in the task frame
  const Eigen::Matrix<double, 6, 1> affordance_twist =
    calculateAffordanceTwist(req.screw, theta_dot);
  const Eigen::Matrix<double, 6, 1> affordance_wrench = calculateAffordanceWrench(
    req.screw, affordance_twist, req.task_impedance_translation, req.task_impedance_rotation);

  // Convert twist in task frame to be twist in moving frame
  const Eigen::Matrix<double, 6, 1> moving_twist =
    transformTwist(affordance_twist, tf_moving_to_task);

  // Calculate the wrench to apply in the moving frame
  const Eigen::Matrix<double, 6, 1> moving_wrench =
    calculateAppliedWrench(affordance_wrench, tf_moving_to_task, req.screw);

  // Package for response
  feedback.moving_frame_twist.header.frame_id = tfmsg_moving_to_task->header.frame_id;
  feedback.moving_frame_twist.twist = tf2::toMsg(moving_twist);
  feedback.expected_wrench.header.frame_id = tfmsg_moving_to_task->header.frame_id;
  feedback.expected_wrench.wrench = VectorToWrench(moving_wrench);
  feedback.tf_moving_to_task = *tfmsg_moving_to_task;

  return true;
}

std::optional<AffordanceTrajectory> APScrewExecutor::getTrajectoryCommands(
  const AffordancePrimitiveGoal & req, size_t num_steps)
{
  // Lookup or check passed TF info
  std::optional<TransformStamped> tfmsg_moving_to_task = getTFInfo(req);
  if (!tfmsg_moving_to_task.has_value()) {
    return std::nullopt;
  }
  Eigen::Isometry3d tf_moving_to_task = tf2::transformToEigen(*tfmsg_moving_to_task);

  // Set the sign of the velocity cmd to be same as requested delta
  const double theta_dot = copysign(req.theta_dot, req.screw_distance);

  // Calculate the twist and wrench of the affordance frame
  const Eigen::Matrix<double, 6, 1> affordance_twist =
    calculateAffordanceTwist(req.screw, theta_dot);
  const Eigen::Matrix<double, 6, 1> affordance_wrench = calculateAffordanceWrench(
    req.screw, affordance_twist, req.task_impedance_translation, req.task_impedance_rotation);

  // Convert these to be the initial moving frame and find the wrench to apply
  // The twist and applied wrench remains constant (in the moving frame) through the motion
  // So later we only need to rotate these to be in the affordance frame
  const Eigen::Matrix<double, 6, 1> initial_moving_twist =
    transformTwist(affordance_twist, tf_moving_to_task);
  const Eigen::Matrix<double, 6, 1> applied_wrench =
    calculateAppliedWrench(affordance_wrench, tf_moving_to_task, req.screw);

  // Use ScrewAxis to generate a list of waypoints (defined w.r.t. affordance frame)
  const double theta_step = req.screw_distance / num_steps;
  ScrewAxis screw_axis;
  screw_axis.setScrewAxis(req.screw);
  std::vector<Eigen::Isometry3d> waypoints = screw_axis.getWaypoints(theta_step, num_steps);

  // Set up output
  AffordanceTrajectory ap_trajectory;
  ap_trajectory.trajectory.reserve(num_steps + 1);

  // Put all the information in the task frame
  ap_trajectory.header.frame_id = tfmsg_moving_to_task->child_frame_id;

  // Fill pose, twist, wrench info from generated waypoints
  const double time_step = theta_step / theta_dot;
  double this_time = 0;
  for (auto wp : waypoints) {
    // Set up this waypoint
    AffordanceWaypoint this_waypoint;
    std::chrono::duration<double> waypoint_duration(this_time);
    this_waypoint.time_from_start =
      rclcpp::Duration(std::chrono::duration_cast<std::chrono::nanoseconds>(waypoint_duration));
    this_time += time_step;

    // Find the pose in the task frame as:
    // task_to_now = waypoint_in_affordance_frame * affordance_to_moving
    // We consider affordance_to_moving constant and the inverse of given tf_moving_to_task
    Eigen::Isometry3d tf_task_to_this_wp = wp * tf_moving_to_task.inverse();
    this_waypoint.pose = tf2::toMsg(tf_task_to_this_wp);

    // Now rotate the twist and wrench vectors to be in the affordance frame
    Eigen::MatrixXd rotator(6, 6);
    rotator.setZero();
    rotator.block<3, 3>(0, 0) = tf_task_to_this_wp.linear();
    rotator.block<3, 3>(3, 3) = tf_task_to_this_wp.linear();

    const Eigen::Matrix<double, 6, 1> this_twist = rotator * initial_moving_twist;
    this_waypoint.twist = tf2::toMsg(this_twist);

    const Eigen::Matrix<double, 6, 1> this_wrench = rotator * applied_wrench;
    this_waypoint.wrench = VectorToWrench(this_wrench);

    ap_trajectory.trajectory.push_back(this_waypoint);
  }

  // We should set the twist/wrench of the last waypoint to be 0
  ap_trajectory.trajectory.back().twist = Twist();
  ap_trajectory.trajectory.back().wrench = Wrench();

  return ap_trajectory;
}

std::optional<TransformStamped> APScrewExecutor::getTFInfo(const AffordancePrimitiveGoal & req)
{
  TransformStamped tfmsg_moving_to_task_frame;

  // Check if we can transform the Task frame to the Moving frame
  if (req.moving_frame_source == req.LOOKUP) {
    // Lookup the TF
    try {
      tfmsg_moving_to_task_frame = tfBuffer_->lookupTransform(
        req.moving_frame_name, req.screw.header.frame_id, rclcpp::Time(0));
    } catch (tf2::TransformException & ex) {
      rclcpp::Clock & clock = *node_->get_clock();
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), clock, std::chrono::milliseconds(1000).count(), "%s", ex.what());
      return std::nullopt;
    }
  } else if (req.moving_frame_source == req.PROVIDED) {
    // Set it
    tfmsg_moving_to_task_frame = req.moving_to_task_frame;
    // Check validity
    if (tfmsg_moving_to_task_frame.child_frame_id != req.screw.header.frame_id) {
      rclcpp::Clock & clock = *node_->get_clock();
      RCLCPP_WARN_STREAM_THROTTLE(
        node_->get_logger(), clock, std::chrono::milliseconds(1000).count(),
        "Provided 'moving_to_task_frame' child frame "
        "name does not match screw header (task) frame, ending...");
      return std::nullopt;
    }
  } else {
    rclcpp::Clock & clock = *node_->get_clock();
    RCLCPP_WARN_STREAM_THROTTLE(
      node_->get_logger(), clock, std::chrono::milliseconds(1000).count(),
      "Unexpected 'moving_frame_source' requested, ending...");
    return std::nullopt;
  }

  // Set the sign of the velocity cmd to be same as requested delta
  const double theta_dot = copysign(req.theta_dot, req.screw_distance);

  // Calculate commanded twist in Task frame
  ScrewAxis screw_axis;
  screw_axis.setScrewAxis(req.screw);
  TwistStamped twist_in_task_frame = screw_axis.getTwist(theta_dot);

  // Convert twist to EE frame using the adjoint
  Eigen::Matrix<double, 6, 1> eigen_twist_task_frame;
  tf2::fromMsg(twist_in_task_frame.twist, eigen_twist_task_frame);

  // Figure out estimated wrench
  Eigen::Matrix<double, 6, 1> eigen_wrench_task_frame;
  if (req.screw.is_pure_translation) {
    eigen_wrench_task_frame.head(3) =
      req.task_impedance_translation * eigen_twist_task_frame.head(3);
    eigen_wrench_task_frame.tail(3).setZero();
  } else {
    eigen_wrench_task_frame.head(3) =
      req.task_impedance_translation * req.screw.pitch * eigen_twist_task_frame.tail(3);
    eigen_wrench_task_frame.tail(3) = req.task_impedance_rotation * eigen_twist_task_frame.tail(3);
  }

  return tfmsg_moving_to_task_frame;
}
}  // namespace affordance_primitives
