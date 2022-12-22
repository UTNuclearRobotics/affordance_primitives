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
#include <algorithm>
namespace affordance_primitives
{

/**
 * @brief 
 *
 * @param tf_m_to_q Forward kinematics of the robot
 * @param tf_m_to_s Start pose
 * @param screw_axis_set Set of screw axes (S1,...Sm) in frame M
 * @param phi_max Set of screw distances to move
 * @param phi_guess Set of initial random guesses for screw angles
 * @param phi_out Set of screw angles that satisfy the constraint function
 *
 * @return 
 */
bool constraintFn(
  const Eigen::Isometry3d & tf_m_to_q, const Eigen::Isometry3d & tf_m_to_s, const std::vector<ScrewAxis> & screw_axis_set, const Eigen::VectorXd phi_max, const Eigen::VectorXd phi_start,
  Eigen::Ref<Eigen::VectorXd> phi_out);

Eigen::Isometry3d productOfExponentials (const std::vector<ScrewAxis>& screwAxisSet, const Eigen::VectorXd phi, int size, int start, int end);

Eigen::VectorXd errorDerivative(
  const Eigen::Isometry3d & tf_q_to_m, const Eigen::Isometry3d & tf_m_to_s,
  const Eigen::VectorXd phi_current, const std::vector<ScrewAxis>& screw_axis_set);

Eigen::VectorXd eta (const Eigen::Isometry3d & tf);

Eigen::VectorXd clamp(const Eigen::VectorXd arr, const int size, const Eigen::VectorXd arr_high);
}  // namespace affordance_primitives


