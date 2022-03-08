#include <affordance_primitives/screw_model/affordance_utils.hpp>
#include <affordance_primitives/screw_model/screw_axis.hpp>
#include <affordance_primitives/screw_model/screw_execution.hpp>

namespace affordance_primitives
{
APScrewExecutor::APScrewExecutor() : tfListener_(tfBuffer_)
{
}

APScrewExecutor::APScrewExecutor(ros::NodeHandle& nh, const std::string& server_name_) : tfListener_(tfBuffer_)
{
  screw_twist_server_ =
      nh.advertiseService(ros::names::append(nh.getNamespace(), server_name_), &APScrewExecutor::getScrewTwist, this);
}

bool APScrewExecutor::getScrewTwist(AffordancePrimitive::Request& req, AffordancePrimitive::Response& res)
{
  TransformStamped tfmsg_moving_to_task_frame;

  // Check if we can transform the Task frame to the Moving frame
  if (req.moving_frame_source == req.LOOKUP)
  {
    // Lookup the TF
    try
    {
      tfmsg_moving_to_task_frame =
          tfBuffer_.lookupTransform(req.moving_frame_name, req.screw.header.frame_id, ros::Time(0));
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN_THROTTLE(1, "%s", ex.what());
      return false;
    }
  }
  else if (req.moving_frame_source == req.PROVIDED)
  {
    // Set it
    tfmsg_moving_to_task_frame = req.moving_to_task_frame;
    // Check validity
    if (tfmsg_moving_to_task_frame.child_frame_id != req.screw.header.frame_id)
    {
      ROS_WARN_STREAM_THROTTLE(1, "Provided 'moving_to_task_frame' child frame "
                                  "name does not match screw header (task) frame, ending...");
      return false;
    }
  }
  else
  {
    ROS_WARN_STREAM_THROTTLE(1, "Unexpected 'moving_frame_source' requested, ending...");
    return false;
  }

  // Calculate commanded twist in Task frame
  ScrewAxis screw_axis;
  screw_axis.setScrewAxis(req.screw);
  TwistStamped twist_in_task_frame = screw_axis.getTwist(req.theta_dot);

  // Convert twist to EE frame using the adjoint
  Eigen::Matrix<double, 6, 1> eigen_twist_task_frame;
  tf2::fromMsg(twist_in_task_frame.twist, eigen_twist_task_frame);
  Eigen::Matrix<double, 6, 1> eigen_twist_moving_frame =
      getAdjointMatrix(tfmsg_moving_to_task_frame.transform) * eigen_twist_task_frame;

  // Figure out estimated wrench
  Wrench wrench_in_task_frame;
  wrench_in_task_frame.force.x = req.task_impedance.trans_x * twist_in_task_frame.twist.linear.x;
  wrench_in_task_frame.force.y = req.task_impedance.trans_y * twist_in_task_frame.twist.linear.y;
  wrench_in_task_frame.force.z = req.task_impedance.trans_z * twist_in_task_frame.twist.linear.z;
  wrench_in_task_frame.torque.x = req.task_impedance.rot_x * twist_in_task_frame.twist.angular.x;
  wrench_in_task_frame.torque.y = req.task_impedance.rot_y * twist_in_task_frame.twist.angular.y;
  wrench_in_task_frame.torque.z = req.task_impedance.rot_z * twist_in_task_frame.twist.angular.z;

  // Convert to Eigen type
  Eigen::Vector3d eig_torque, eig_force;
  tf2::fromMsg(wrench_in_task_frame.torque, eig_torque);
  tf2::fromMsg(wrench_in_task_frame.force, eig_force);

  Eigen::VectorXd eig_wrench_in_task_frame(6);
  eig_wrench_in_task_frame.tail(3) = eig_torque;
  eig_wrench_in_task_frame.head(3) = eig_force;

  // Convert wrench to moving frame
  Eigen::Isometry3d tf_eigen_moving_to_task_frame = tf2::transformToEigen(tfmsg_moving_to_task_frame);
  Eigen::VectorXd eigen_wrench_moving_frame =
      getAdjointMatrix(tf_eigen_moving_to_task_frame.inverse()).transpose() * eig_wrench_in_task_frame;

  Eigen::Vector3d moment = eigen_wrench_moving_frame.head(3);
  Eigen::Vector3d screw_origin;
  tf2::fromMsg(req.screw.origin, screw_origin);
  Eigen::Vector3d radius =
      tf_eigen_moving_to_task_frame.translation() + tf_eigen_moving_to_task_frame.linear() * screw_origin;
  Eigen::Vector3d force_in_moving_frame = radius.cross(moment);

  // Package for response
  res.moving_frame_twist.header.frame_id = tfmsg_moving_to_task_frame.header.frame_id;
  res.moving_frame_twist.twist = tf2::toMsg(eigen_twist_moving_frame);
  res.expected_wrench.header.frame_id = tfmsg_moving_to_task_frame.header.frame_id;
  res.expected_wrench.wrench.force.x = force_in_moving_frame.x();
  res.expected_wrench.wrench.force.y = force_in_moving_frame.y();
  res.expected_wrench.wrench.force.z = force_in_moving_frame.z();

  return true;
}
}  // namespace affordance_primitives
