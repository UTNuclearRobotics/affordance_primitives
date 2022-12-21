///////////////////////////////////////////////////////////////////////////////
//      Title     : screw_planning.hpp
//      Project   : affordance_primitives
//      Created   : 06/24/2022
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

#include <moveit_msgs/Constraints.h>

#include <affordance_primitives/screw_model/affordance_utils.hpp>
#include <affordance_primitives/screw_model/screw_axis.hpp>
#include <queue>
#include <utility>

namespace affordance_primitives
{
//1D screw axes
bool constraintFn(
  const Eigen::Isometry3d & current_pose, const Eigen::Isometry3d & start_pose,
  const ScrewAxis & screw_axis, const std::pair<double, double> theta_limits, double theta_guess,
  Eigen::Ref<Eigen::VectorXd> out);

/** Calculates the error vector from a given transformation
   *
   * @param tf_err The transformation between two poses
   * @return A vector representing the error where the first 3 values are the 
   * translation, and the last 3 are the orientation error in axis-angle format
   */
Eigen::VectorXd calcError(const Eigen::Isometry3d & tf_err);

double calcErrorDerivative(
  const Eigen::Isometry3d & tf_m_to_q, const Eigen::Isometry3d & tf_m_to_e,
  const double current_theta, const ScrewAxis & screw_axis);

/** Runs gradient descent one time
   *
   * @param tf_m_to_q The TF from planning frame to the pose to check
   * @param tf_m_to_e The TF from planning frame to the starting path pose (when theta = 0)
   * @param theta_start The theta to start from
   * @param theta_limits The (lower, upper) bounds on theta
   * @param screw_axis The screw axis defining the path
   * @return First: the found theta. Second: the TF from passed pose to closest point on path
   */
std::pair<double, Eigen::Isometry3d> runGradientDescent(
  const Eigen::Isometry3d & tf_m_to_q, const Eigen::Isometry3d & tf_m_to_e,
  const double theta_start, const double theta_max, const ScrewAxis & screw_axis);

//multi-dimensional screw axes
bool constraintFn(
  const Eigen::Isometry3d & tf_m_to_q, const Eigen::Isometry3d & tf_m_to_s,
  const std::vector<ScrewAxis> & screwAxisSet, std::vector<double> phi_max,
  std::vector<double> phi_guess, Eigen::Ref<Eigen::VectorXd> phi_out);

Eigen::Isometry3d productOfExponentials (const std::vector<Eigen::VectorXd>& screwAxisSet, const std::vector<double> phi, int size, int start, int end);

double calcErrorDerivative(
  const Eigen::Isometry3d & tf_m_to_q, const Eigen::Isometry3d & tf_m_to_s,
  const std::vector<double> current_phi, const std::vector<Eigen::VectorXd> & screw_axis_set);

Eigen::VectorXd eta(const Eigen::Isometry3d & tf);
}  // namespace affordance_primitives
