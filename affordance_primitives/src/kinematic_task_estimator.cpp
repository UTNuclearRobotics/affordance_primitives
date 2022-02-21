#include <affordance_primitives/kinematic_task_estimator.hpp>
#include <tf2_eigen/tf2_eigen.h>

namespace affordance_primitives
{
KinematicTaskEstimator::KinematicTaskEstimator() : tfListener_(tfBuffer_)
{
}

void KinematicTaskEstimator::initialize(const ros::NodeHandle& nh)
{
  nh_ = nh;
}

bool KinematicTaskEstimator::resetTaskEstimation(double reset_val)
{
  current_estimation_ = reset_val;
  last_tf_moving_to_task_frame_.reset();
  return true;
}

std::optional<double>
KinematicTaskEstimator::estimateTaskAngle(const affordance_primitives::AffordancePrimitive::Request& ap_req)
{
  TransformStamped tfmsg_moving_to_task_frame;

  // Check if we can transform the Task frame to the Moving frame
  if (ap_req.moving_frame_source == ap_req.LOOKUP)
  {
    // Lookup the TF
    try
    {
      tfmsg_moving_to_task_frame =
          tfBuffer_.lookupTransform(ap_req.moving_frame_name, ap_req.screw.header.frame_id, ros::Time(0));
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN_THROTTLE(1, "%s", ex.what());
      return std::nullopt;
    }
  }
  else if (ap_req.moving_frame_source == ap_req.PROVIDED)
  {
    // Set it
    tfmsg_moving_to_task_frame = ap_req.moving_to_task_frame;
    // Check validity
    if (tfmsg_moving_to_task_frame.child_frame_id != ap_req.screw.header.frame_id)
    {
      ROS_WARN_STREAM_THROTTLE(1, "Provided 'moving_to_task_frame' child frame "
                                  "name does not match screw header (task) frame, ending...");
      return std::nullopt;
    }
  }
  else
  {
    ROS_WARN_STREAM_THROTTLE(1, "Unexpected 'moving_frame_source' requested, ending...");
    return std::nullopt;
  }

  // Check if we HAVE a previous pose. If not, just return the current estimate
  if (!last_tf_moving_to_task_frame_)
  {
    last_tf_moving_to_task_frame_ = tfmsg_moving_to_task_frame;
    return current_estimation_;
  }

  // Calculate the delta: find the TF from last to current, then take the matrix log on this
  // Convert to Eigen types
  const Eigen::Isometry3d last_tf = tf2::transformToEigen(last_tf_moving_to_task_frame_.value());
  const Eigen::Isometry3d current_tf = tf2::transformToEigen(tfmsg_moving_to_task_frame);

  // Get TF: last to current
  const Eigen::Isometry3d tf_last_to_current = last_tf * current_tf.inverse();

  // Translation case is much easier
  if (ap_req.screw.is_pure_translation) {
    current_estimation_ += tf_last_to_current.translation().norm();
  }
  else {
    // Use Eigen to do the heavy math lifting
    Eigen::AngleAxisd rotation_se3(tf_last_to_current.linear());

    // Update the estimate and return
    current_estimation_ += rotation_se3.angle();
  }

  // Save last pose
  last_tf_moving_to_task_frame_ = tfmsg_moving_to_task_frame;

  return current_estimation_;
}

}  // namespace affordance_primitives

PLUGINLIB_EXPORT_CLASS(affordance_primitives::KinematicTaskEstimator, affordance_primitives::TaskEstimator);
