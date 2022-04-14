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

#include <actionlib/client/simple_action_client.h>
#include <affordance_primitive_msgs/AffordancePrimitiveAction.h>
#include <ros/ros.h>

#include <affordance_primitives/ap_common.hpp>

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "ap_client_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(0);
  spinner.start();

  // This is the name of the action that will be available for executing Affordance Primitives
  const std::string action_name = "ap_execution";

  // Set up the action client. This will be the interface with the AP execution server
  auto action_client = std::make_unique<
    actionlib::SimpleActionClient<affordance_primitive_msgs::AffordancePrimitiveAction>>(
    action_name, true);
  if (!action_client->waitForServer(ros::Duration(30))) {
    return EXIT_FAILURE;
  }

  // Set up the action request
  // This consists of information about the EE link, the screw axis to move about, the task's impedance, robot control
  // parameters to use, and how fast and far to move
  affordance_primitive_msgs::AffordancePrimitiveGoal ap_goal;

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
  action_client->sendGoal(ap_goal);

  // We can interact with the client as usual
  bool finished_before_timeout = action_client->waitForResult(ros::Duration(15));
  if (finished_before_timeout) {
    auto result = action_client->getResult();
    ROS_ERROR_STREAM("Result: " << affordance_primitives::to_string(result->result));
  } else {
    action_client->cancelGoal();
    ROS_ERROR("Action did not finish before the time out.");
  }
}
