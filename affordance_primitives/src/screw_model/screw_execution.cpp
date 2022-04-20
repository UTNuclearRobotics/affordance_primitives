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
  TransformStamped tfmsg_moving_to_task_frame;

  // Check if we can transform the Task frame to the Moving frame
  if (req.moving_frame_source == req.LOOKUP) {
    // Lookup the TF
    try {
      tfmsg_moving_to_task_frame =
        tfBuffer_.lookupTransform(req.moving_frame_name, req.screw.header.frame_id, ros::Time(0));
    } catch (tf2::TransformException & ex) {
      ROS_WARN_THROTTLE(1, "%s", ex.what());
      return false;
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
      return false;
    }
  } else {
    ROS_WARN_STREAM_THROTTLE(1, "Unexpected 'moving_frame_source' requested, ending...");
    return false;
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
  Eigen::Matrix<double, 6, 1> eigen_twist_moving_frame =
    getAdjointMatrix(tfmsg_moving_to_task_frame.transform) * eigen_twist_task_frame;

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

  // Convert wrench to moving frame
  Eigen::Isometry3d tf_eigen_moving_to_task_frame =
    tf2::transformToEigen(tfmsg_moving_to_task_frame);
  Eigen::Matrix<double, 6, 1> eigen_wrench_moving_frame =
    getAdjointMatrix(tf_eigen_moving_to_task_frame.inverse()).transpose() * eigen_wrench_task_frame;

  // Calculate wrench to apply
  Eigen::Matrix<double, 6, 1> wrench_to_apply;
  wrench_to_apply.tail(3) = eigen_wrench_moving_frame.tail(3);
  Eigen::Vector3d screw_origin;
  tf2::fromMsg(req.screw.origin, screw_origin);
  Eigen::Vector3d radius = tf_eigen_moving_to_task_frame.translation() +
                           tf_eigen_moving_to_task_frame.linear() * screw_origin;
  wrench_to_apply.head(3) = eigen_wrench_moving_frame.head(3) +
                            radius.cross(Eigen::Vector3d(eigen_wrench_moving_frame.tail(3)));

  // Package for response
  feedback.moving_frame_twist.header.frame_id = tfmsg_moving_to_task_frame.header.frame_id;
  feedback.moving_frame_twist.twist = tf2::toMsg(eigen_twist_moving_frame);
  feedback.expected_wrench.header.frame_id = tfmsg_moving_to_task_frame.header.frame_id;
  feedback.expected_wrench.wrench = VectorToWrench(wrench_to_apply);
  feedback.tf_moving_to_task = tfmsg_moving_to_task_frame;

  return true;
}
}  // namespace affordance_primitives
