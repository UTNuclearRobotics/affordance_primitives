#include <affordance_primitives/screw_execution.hpp>
#include <affordance_primitives/affordance_utils.hpp>
#include <affordance_primitives/screw_axis.hpp>

namespace affordance_primitives
{
APScrewExecutor::APScrewExecutor() : tfBuffer_(), tfListener_(tfBuffer_)
{
}

APScrewExecutor::APScrewExecutor(ros::NodeHandle& nh, const std::string& server_name_)
  : tfBuffer_(), tfListener_(tfBuffer_)
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
  Eigen::VectorXd eigen_twist_moving_frame =
      getAdjointMatrix(tfmsg_moving_to_task_frame.transform) * twistToVector(twist_in_task_frame.twist);

  // Package for response
  res.moving_frame_twist.header.frame_id = tfmsg_moving_to_task_frame.header.frame_id;
  res.moving_frame_twist.twist = vectorToTwist(eigen_twist_moving_frame);

  return true;
}
}  // namespace affordance_primitives
