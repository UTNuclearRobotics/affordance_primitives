///////////////////////////////////////////////////////////////////////////////
//      Title     : task_monitor.hpp
//      Project   : affordance_primitives
//      Created   : 02/05/2021
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

#include <ros/ros.h>
#include <affordance_primitives/ap_common.hpp>
#include <affordance_primitives/msg_types.hpp>

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <optional>
#include <queue>
#include <thread>

namespace affordance_primitives
{
/**
 * Monitors status during an AP execution
 *
 * Can block or not while monitoring
 */
class TaskMonitor
{
public:
  TaskMonitor(const ros::NodeHandle& nh, const std::string ft_topic_name);

  ~TaskMonitor();

  /**
   * This starts monitoring, but doesn't block
   *
   * @param parameters Parameters used for monitor F/T and Kinematic constraints
   * @param timeout Timeout to wait, a non-positive number is infinite timeout.
   * Timing out is considered a SUCCESS from the TaskMonitor's perspective
   */
  void startMonitor(const APRobotParameter& parameters, const double timeout = 0.0);

  /**
   * Blocks until a result is found
   * @return The result after waiting
   */
  AffordancePrimitiveResult waitForResult();

  /**
   * Grabs the result of the monitoring
   * @return The result. If there is none, a std::nullopt is returned
   */
  std::optional<AffordancePrimitiveResult> getResult();

  /**
   * Cancels the monitor
   */
  void stopMonitor();

private:
  /**
   * Main function where the monitoring happens
   */
  void mainLoop();

  // node handle
  ros::NodeHandle nh_;

  // Mutex for protecting the class variables
  mutable std::mutex mutex_;
  std::condition_variable condition_variable_;

  // An end time for the timeout
  ros::Time end_time_;

  // We need a result to report at the end
  AffordancePrimitiveResult result_;

  // Hold onto parameters during a monitor session
  APRobotParameter parameters_;

  // Track monitor status and if a stop has been requested
  std::atomic<bool> active_monitor_{ false };
  std::atomic<bool> continue_monitoring_{ false };
  std::atomic<bool> has_timeout_{ false };

  // Thread where the actual monitoring will happen
  std::thread thread_;

  // Subscriber for listening to F/T topic
  ros::Subscriber ft_sub_;

  // F/T Callback
  void ftCB(const WrenchStamped& msg);

  // Store recent wrench message
  std::queue<WrenchStamped> last_wrench_msgs_;

  // Check if AP is violated
  std::optional<AffordancePrimitiveResult> checkForViolaton(const APRobotParameter& params);
};
}  // namespace affordance_primitives
