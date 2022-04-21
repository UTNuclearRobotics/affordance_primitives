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

APScrewExecutor::APScrewExecutor() : tfListener_(tfBuffer_) {}

bool APScrewExecutor::getScrewTwist(
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

std::vector<AffordanceWaypoint> APScrewExecutor::getScrewWaypoints(
  const AffordancePrimitiveGoal & req, size_t num_steps)
{
  // Lookup or check passed TF info
  std::optional<TransformStamped> tfmsg_moving_to_task = getTFInfo(req);
  if (!tfmsg_moving_to_task.has_value()) {
    return std::vector<AffordanceWaypoint>();
  }
  Eigen::Isometry3d tf_moving_to_task = tf2::transformToEigen(*tfmsg_moving_to_task);

  // Set the sign of the velocity cmd to be same as requested delta
  const double theta_dot = copysign(req.theta_dot, req.screw_distance);

  // Transform screw to be in initial moving frame
  ScrewStamped screw_moving_frame;
  TransformStamped tfmsg_task_to_moving = tf2::eigenToTransform(tf_moving_to_task.inverse());
  tfmsg_task_to_moving.header.frame_id = tfmsg_moving_to_task->child_frame_id;
  tfmsg_task_to_moving.child_frame_id = tfmsg_moving_to_task->header.frame_id;
  try {
    screw_moving_frame = transformScrew(req.screw, tfmsg_task_to_moving);
  } catch (std::runtime_error) {
    return std::vector<AffordanceWaypoint>();
  }

  // Calculate the twist and wrench in the initial moving frame
  const Eigen::Matrix<double, 6, 1> affordance_twist =
    calculateAffordanceTwist(screw_moving_frame, theta_dot);
  const Eigen::Matrix<double, 6, 1> affordance_wrench = calculateAffordanceWrench(
    screw_moving_frame, affordance_twist, req.task_impedance_translation,
    req.task_impedance_rotation);

  // Use ScrewAxis to generate a list of waypoints (defined w.r.t. initial moving frame)
  const double theta_step = req.screw_distance / num_steps;
  ScrewAxis screw_axis;
  screw_axis.setScrewAxis(screw_moving_frame);
  std::vector<Eigen::Isometry3d> waypoints = screw_axis.getWaypoints(theta_step, num_steps);

  // Set up outputs
  std::vector<AffordanceWaypoint> waypoints_msgs;
  waypoints_msgs.reserve(num_steps + 1);

  // Fill pose, twist, wrench info from generated waypoints
  const double time_step = theta_step / theta_dot;
  double this_time = 0;
  for (auto wp : waypoints) {
    // TODO: The twist/wrench is the same in every waypoint (we have assumed a constant screw axis). Could skip these calculations
    // Convert affordance twist to this frame
    const Eigen::Matrix<double, 6, 1> moving_twist = transformTwist(affordance_twist, wp.inverse());

    // Calculate wrench to apply in this frame
    const Eigen::Matrix<double, 6, 1> moving_wrench =
      calculateAppliedWrench(affordance_wrench, wp.inverse(), screw_moving_frame);

    // Set the outputs
    AffordanceWaypoint this_waypoint;
    this_waypoint.pose.pose = tf2::toMsg(wp);
    this_waypoint.pose.header.frame_id = tfmsg_moving_to_task->header.frame_id;
    this_waypoint.twist = tf2::toMsg(moving_twist);
    this_waypoint.wrench = VectorToWrench(moving_wrench);
    this_waypoint.time_from_start = ros::Duration().fromSec(this_time);
    waypoints_msgs.push_back(this_waypoint);

    // Update the time for the next waypoint
    this_time += time_step;
  }

  // We should set the twist/wrench of the last waypoint to be 0
  waypoints_msgs.back().twist = Twist();
  waypoints_msgs.back().wrench = Wrench();

  return waypoints_msgs;
}

std::optional<TransformStamped> APScrewExecutor::getTFInfo(const AffordancePrimitiveGoal & req)
{
  TransformStamped tfmsg_moving_to_task_frame;

  // Check if we can transform the Task frame to the Moving frame
  if (req.moving_frame_source == req.LOOKUP) {
    // Lookup the TF
    try {
      tfmsg_moving_to_task_frame =
        tfBuffer_.lookupTransform(req.moving_frame_name, req.screw.header.frame_id, ros::Time(0));
    } catch (tf2::TransformException & ex) {
      ROS_WARN_THROTTLE(1, "%s", ex.what());
      return std::nullopt;
    }
  } else if (req.moving_frame_source == req.PROVIDED) {
    // Set it
    tfmsg_moving_to_task_frame = req.moving_to_task_frame;
    // Check validity
    if (tfmsg_moving_to_task_frame.child_frame_id != req.screw.header.frame_id) {
      ROS_WARN_STREAM_THROTTLE(
        1,
        "Provided 'moving_to_task_frame' child frame "
        "name does not match screw header (task) frame, ending...");
      return std::nullopt;
    }
  } else {
    ROS_WARN_STREAM_THROTTLE(1, "Unexpected 'moving_frame_source' requested, ending...");
    return std::nullopt;
  }

  return tfmsg_moving_to_task_frame;
}
}  // namespace affordance_primitives
