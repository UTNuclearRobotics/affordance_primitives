///////////////////////////////////////////////////////////////////////////////
//      Title     : ap_client_node.cpp
//      Project   : ap_examples
//      Created   : 03/31/2022
//      Author    : Adam Pettinger
//      Copyright : Copyright© The University of Texas at Austin, 2014-2022. All
//      rights reserved.
//
//          All files within this directory are subject to the following, unless
//          an alternative license is explicitly included within the text of
//          each file.
//
//          This software and documentation constitute an unpublished work
//          and contain valuable trade secrets and proprietary information
//          belonging to the University. None of the foregoing material may be
//          copied or duplicated or disclosed without the express, written
//          permission of the University. THE UNIVERSITY EXPRESSLY DISCLAIMS ANY
//          AND ALL WARRANTIES CONCERNING THIS SOFTWARE AND DOCUMENTATION,
//          INCLUDING ANY WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
//          PARTICULAR PURPOSE, AND WARRANTIES OF PERFORMANCE, AND ANY WARRANTY
//          THAT MIGHT OTHERWISE ARISE FROM COURSE OF DEALING OR USAGE OF TRADE.
//          NO WARRANTY IS EITHER EXPRESS OR IMPLIED WITH RESPECT TO THE USE OF
//          THE SOFTWARE OR DOCUMENTATION. Under no circumstances shall the
//          University be liable for incidental, special, indirect, direct or
//          consequential damages or loss of profits, interruption of business,
//          or related expenses which may arise from use of software or
//          documentation, including but not limited to those resulting from
//          defects in software and/or documentation, or loss or inaccuracy of
//          data of any kind.
//
///////////////////////////////////////////////////////////////////////////////

#include <affordance_primitive_msgs/action/affordance_primitive.hpp>
#include <affordance_primitives/ap_common.hpp>
#include <affordance_primitives/screw_model/affordance_utils.hpp>
#include <rclcpp/rclcpp.hpp>

#include "rclcpp_action/rclcpp_action.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("ap_client_node");

  // This is the name of the action that will be available for executing Affordance Primitives
  const std::string action_name = "ap_execution";

  // Set up the action client. This will be the interface with the AP execution server
  using AffordancePrimitive = affordance_primitive_msgs::action::AffordancePrimitive;
  using GoalHandleAffordancePrimitive = rclcpp_action::ClientGoalHandle<AffordancePrimitive>;
  auto action_client = rclcpp_action::create_client<AffordancePrimitive>(node, action_name);

  // Set up the action request
  // This consists of information about the EE link, the screw axis to move about, the task's impedance, robot control
  // parameters to use, and how fast and far to move
  AffordancePrimitive::Goal ap_goal;

  // There are 2 options for EE link information. One is a link name to look up on the TF tree. Here we will directly
  // pass the transformation to the link
  ap_goal.moving_frame_source = ap_goal.PROVIDED;
  ap_goal.moving_to_task_frame.header.frame_id = "ee_link";
  ap_goal.moving_to_task_frame.child_frame_id = "task_frame";
  ap_goal.moving_to_task_frame.transform.translation.x = 1.0;
  ap_goal.moving_to_task_frame.transform.translation.y = -0.5;

  // Next we set the screw axis
  // The moving frame will rotate about this axis while simultaneously translating along it
  ap_goal.screw.header.frame_id = "task_frame";
  ap_goal.screw.is_pure_translation = false;
  ap_goal.screw.origin.x = 0.25;
  ap_goal.screw.origin.z = 1.0;
  ap_goal.screw.axis.y = 0.5;
  ap_goal.screw.axis.z = 0.5;
  ap_goal.screw.pitch = 0.01;

  // The task impedance is the force and torque required to move the object at some velocity
  ap_goal.task_impedance_translation = 20;
  ap_goal.task_impedance_rotation = 400;

  // An AP provides an opportunity to set some control parameters for the robot.
  // These include addmittance, force/torque limits, and velocity limits
  ap_goal.robot_params.admittance.trans_x = 0.25;
  ap_goal.robot_params.max_force = 100;
  ap_goal.robot_params.max_torque = 10;
  ap_goal.robot_params.max_wrench.trans_x = 75;
  ap_goal.robot_params.max_ee_velocity.rot_y = 0.25;

  // Finally, we request how far to move and how quickly
  ap_goal.screw_distance = 0.5 * M_PI;
  ap_goal.theta_dot = 0.05;

  // To trigger the move, we call the action
  auto send_goal_options = rclcpp_action::Client<AffordancePrimitive>::SendGoalOptions();
  auto goal_response_callback =
    [&node](std::shared_future<GoalHandleAffordancePrimitive::SharedPtr> future) {
      auto goal_handle = future.get();
      if (!goal_handle) {
        RCLCPP_ERROR(node->get_logger(), "Goal was rejected by server");
      } else {
        RCLCPP_INFO(node->get_logger(), "Goal accepted by server, waiting for result");
      }
    };
  auto feedback_callback = [&node](
                             GoalHandleAffordancePrimitive::SharedPtr,
                             const std::shared_ptr<const AffordancePrimitive::Feedback> feedback) {
    // You could send these commands directly to the robot, filter them, etc

    // Here we will just print them out
    std::string output = affordance_primitives::twistToStr(feedback->moving_frame_twist);
    rclcpp::Clock & clock = *node->get_clock();
    RCLCPP_INFO_STREAM_THROTTLE(
      node->get_logger(), clock, std::chrono::milliseconds(1000).count(), output);
  };
  auto result_callback = [&node](const GoalHandleAffordancePrimitive::WrappedResult & result_msg) {
    switch (result_msg.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(node->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(node->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(node->get_logger(), "Unknown result code");
        return;
    }
    std::string output = affordance_primitives::to_string(result_msg.result->result);
    RCLCPP_INFO_STREAM(node->get_logger(), output);
    rclcpp::shutdown();
  };
  send_goal_options.goal_response_callback = goal_response_callback;
  send_goal_options.feedback_callback = feedback_callback;
  send_goal_options.result_callback = result_callback;

  action_client->async_send_goal(ap_goal, send_goal_options);

  rclcpp::spin(node);
}
