///////////////////////////////////////////////////////////////////////////////
//      Title     : ap_executor.hpp
//      Project   : affordance_primitives
//      Created   : 01/21/2021
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

#pragma once

#include <actionlib/server/simple_action_server.h>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>

#include <affordance_primitives/ap_common.hpp>
#include <affordance_primitives/configs_interface/parameter_manager.hpp>
#include <affordance_primitives/msg_types.hpp>
#include <affordance_primitives/screw_model/screw_execution.hpp>
#include <affordance_primitives/task_estimator/task_estimator.hpp>
#include <affordance_primitives/task_monitor/task_monitor.hpp>
#include <mutex>
#include <optional>

namespace affordance_primitives
{
enum Mode { INACTIVE, EXECUTING };

/**
 * Stores parameters needed for execution of APs
 */
struct ExecutorParameters
{
  // Plugins
  std::string param_manager_plugin_name;
  std::string task_estimator_plugin_name;

  // Topics
  std::string monitor_ft_topic_name;
};

/** Changes AP feedback to be within limits given in goal
 *
 * @param params The parameters containing the limits
 * @param feedback The feedback that will be changed to be within limits
 * @param epsilon Range within which to consider 0, or limit unset
 */
void enforceAPLimits(
  const APRobotParameter & params, AffordancePrimitiveFeedback & feedback,
  const double epsilon = 1e-8);

/**
 * Manages the execution of an Affordance Primitive as an Action. A client will need to interface with this class to
 * actually run an AP
 *
 */
class APExecutor
{
public:
  /** Constructor
   *
   * @param nh ROS node handle
   * @param action_name the name the AP executor action will take
   */
  APExecutor(const ros::NodeHandle & nh, const std::string action_name);

  ~APExecutor() { stop(); };

  /** Sets up and starts the action server
   *
   * @param params A number of parameters necessary for execution
   * @return true if successfully initialized, false otherwise
   */
  bool initialize(const ExecutorParameters & params);

  /* \brief Stops the current action */
  void stop();

protected:
  /** Performs a single Affordance Primitive screw motion
   *
   * @param ap_goal The screw-primitive to move with
   * @return The result of the execution
   */
  AffordancePrimitiveResult execute(
    const affordance_primitive_msgs::AffordancePrimitiveGoalConstPtr & goal);

  /** Carries out the setting of the passed parameters
   *
   * @param parameters Parameter object defining this move's parameters
   * @return true if parameters changed successfully, false otherwise
   */
  bool updateParams(const APRobotParameter & parameters);

  /** Does post-execution reset like resetting dims, stopping motion, etc
   *
   * @return True if successful, false otherwise
   */
  bool postExecuteReset();

  // node handle
  ros::NodeHandle nh_;

  // Action server
  actionlib::SimpleActionServer<AffordancePrimitiveAction> action_server_;

  // This manages setting the parameters for a robot
  std::shared_ptr<pluginlib::ClassLoader<ParameterManager>> pm_loader_;
  boost::shared_ptr<ParameterManager> parameter_manager_;

  // This estimates the task angle
  std::shared_ptr<pluginlib::ClassLoader<TaskEstimator>> te_loader_;
  boost::shared_ptr<TaskEstimator> task_estimator_;

  // Hold the current mode of operation
  Mode current_mode_{INACTIVE};
  mutable std::mutex mode_mutex_;

  // The monitor obejct for getting the execution status
  std::unique_ptr<TaskMonitor> monitor_;

  std::unique_ptr<APScrewExecutor> screw_executor_;
};
}  // namespace affordance_primitives
