///////////////////////////////////////////////////////////////////////////////
//      Title     : task_estimator.hpp
//      Project   : affordance_primitives
//      Created   : 02/04/2022
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
#include <affordance_primitives/msg_types.hpp>

#include <optional>

namespace affordance_primitives
{
/**
 * Estimates the state of a task
 */
class TaskEstimator
{
public:
  virtual void initialize(const ros::NodeHandle& nh) = 0;

  /** Returns the estimated angle of a task
   *
   * @param ap_req The affordance primitive being used for motion
   * @return An estimation of the angle (theta) of a screw primitive. The optional is not filled if the input was invalid
   */
  virtual std::optional<double> estimateTaskAngle(const affordance_primitives::AffordancePrimitive::Request& ap_req) = 0;

  /** Resets the internal estimation
   *
   * @param reset_val The value to reset to
   * @return True if successful, false otherwise
   */
  virtual bool resetTaskEstimation(double reset_val = 0.0) = 0;

  virtual ~TaskEstimator(){};

protected:
  TaskEstimator(){};
  ros::NodeHandle nh_;
};
}  // namespace affordance_primitives
