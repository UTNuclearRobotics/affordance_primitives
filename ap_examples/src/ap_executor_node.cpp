///////////////////////////////////////////////////////////////////////////////
//      Title     : ap_executor_node.cpp
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

#include <ros/ros.h>

#include <affordance_primitives/ap_executor/ap_executor.hpp>

// Callback that happens during Execution feedback
void executorFeedbackCB(const affordance_primitive_msgs::AffordancePrimitiveActionFeedback& msg)
{
  // You could send these commands directly to the robot, filter them, etc

  // Here we will just print them out
  ROS_INFO_STREAM_THROTTLE(2.0, msg.feedback);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ap_executor_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(0);
  spinner.start();

  // This is the name of the action that will be available for executing Affordance Primitives
  const std::string action_name = "ap_execution";

  // There are some parameters required for setting up AP execution
  affordance_primitives::ExecutorParameters params;

  // This is the name of the topic that the force/torque sensor publishes to
  // The executor uses it for monitoring the task to make sure contact forces are not too high
  params.monitor_ft_topic_name = "/ft";

  // This is the name of the Robot Configuration Interface plugin
  // It is a C++ class that will be called on to set various execution parameters on the robot
  // You can write your own, or use "EmptyParameterManager" in the affordance_primitives package
  params.param_manager_plugin_name = "ap_examples::ExampleParameterManager";
  // params.param_manager_plugin_name = "affordance_primitives::EmptyParameterManager";

  // This is the name of the Task Estimator plugin
  // It is a C++ class that is called to estimate the screw position/angle as the task executes
  // You can write your own, or use "KinematicTaskEstimator" in the affordance_primitives package
  params.task_estimator_plugin_name = "ap_examples::ExampleTaskEstimator";
  // params.task_estimator_plugin_name = "affordance_primitives::KinematicTaskEstimator";

  // Set up executor
  auto executor = std::make_unique<affordance_primitives::APExecutor>(nh, action_name);

  // Initialize and start action server
  if (!executor->initialize(params))
  {
    return EXIT_FAILURE;
  }

  // Set up subscriber that connects execution feedback to robot commands
  // Every control loop cycle, the executor action server publishes the velocity and force the robot should apply
  ros::Subscriber exeuction_feedback_sub = nh.subscribe(action_name + "/feedback", 1000, executorFeedbackCB);

  ros::waitForShutdown();
}
