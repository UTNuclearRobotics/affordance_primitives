///////////////////////////////////////////////////////////////////////////////
//      Title     : example_task_estimator.hpp
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

#pragma once

#include <affordance_primitives/msg_types.hpp>
#include <affordance_primitives/task_estimator/task_estimator.hpp>
#include <rclcpp/rclcpp.hpp>

namespace ap_examples
{
/**
 * A demonstration TaskEstimator that waits some bit, then estimates PI
 */
class ExampleTaskEstimator : public affordance_primitives::TaskEstimator
{
public:
  ExampleTaskEstimator(){};

  void initialize(rclcpp::Node::SharedPtr node);

  /** Returns the estimated angle of a task
   *
   * @param ap_req The affordance primitive being used for motion
   * @return An estimation of the angle (theta) of a screw primitive. The
   * optional is not filled if the input was invalid
   */
  std::optional<double> estimateTaskAngle(
    const affordance_primitives::AffordancePrimitiveGoal & ap_goal);

  /** Resets the internal estimation
   *
   * @param reset_val The value to reset to
   */
  bool resetTaskEstimation(double reset_val = 0);

private:
  // This example will be time based, so we will save the starting time
  rclcpp::Time start_time_;
};
}  // namespace ap_examples
