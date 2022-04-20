///////////////////////////////////////////////////////////////////////////////
//      Title     : ap_screw_executor.h
//      Project   : affordance_primitives
//      Created   : 10/07/2021
//      Author    : Adam Pettinger
//      Copyright : Copyright© The University of Texas at Austin, 2014-2021. All
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
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>

#include <affordance_primitives/msg_types.hpp>
#include <optional>

namespace affordance_primitives
{
/**
 * @brief Calculates the twist in the affordance frame, with format [linear ; angular]
 * @param screw The screw message
 * @param theta_dot The (signed) velocity to rotate (or translate in linear case) along the screw axis
 * @return Twist defined with respect to the screw axis frame
 */
Eigen::Matrix<double, 6, 1> calculateAffordanceTwist(const ScrewStamped & screw, double theta_dot);

/**
 * @brief Calculates the wrench in the affordance frame, with format [force ; torque]
 * @param screw The screw message
 * @param twist The twist, given in the affordance frame
 * @param impedance_translation Translational impedance from the AP
 * @param impedance_rotation Rotational impedance from the AP
 * @return Wrench defined with respect to the screw axis frame
 */
Eigen::Matrix<double, 6, 1> calculateAffordanceWrench(
  const ScrewStamped & screw, const Eigen::Matrix<double, 6, 1> & twist,
  double impedance_translation, double impedance_rotation);

/**
 * @brief Calculates the wrench needed at the moving frame to perform the task. Given in moving reference frame
 * @param affordance_wrench The calculated "afforance wrench", from the task velocity and impedance
 * @param tf_moving_to_task The transformation from the moving frame to the task frame (i.e. frame screw is defined in)
 * @param screw The screw message for the AP
 * @return Wrench defined with respect to the moving frame, representing the F/T the robot needs to apply
 */
Eigen::Matrix<double, 6, 1> calculateAppliedWrench(
  const Eigen::Matrix<double, 6, 1> & affordance_wrench,
  const Eigen::Isometry3d & tf_moving_to_task, const ScrewStamped & screw);

class APScrewExecutor
{
public:
  APScrewExecutor();

  ~APScrewExecutor(){};

  bool getScrewTwist(const AffordancePrimitiveGoal & req, AffordancePrimitiveFeedback & feedback);

  /**
   * @brief Gets the TF from the moving frame to task frame, either by looking it up or checking the passed one
   * @param req The AP request containing frame information
   * @return If request invalid or lookup failed, returns nullopt. Otherwise, the TF is the location of the task frame with respect to the moving frame
   */
  std::optional<TransformStamped> getTFInfo(const AffordancePrimitiveGoal & req);

private:
  // TF Lookup
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;
};
}  // namespace affordance_primitives
