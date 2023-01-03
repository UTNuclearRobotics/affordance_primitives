///////////////////////////////////////////////////////////////////////////////
//      Title     : screw_planning.hpp
//      Project   : affordance_primitives
//      Created   : 06/24/2022
//      Author    : Janak Panthi and Adam Pettinger
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
#include <algorithm>
#include <queue>
#include <utility>
namespace affordance_primitives
{
//Structs
/**
* @brief Use to set constraint info for screw planning 
*
* @var ScrewConstraintInfo::screw_axis_set
* @brief contains set of screw_axes Set of screw axes (S1,...,Sm) in base frame, M
* @var ScrewConstraintInfo::phi_bounds
* @brief bounds for set of screw angles, phi
* @var ScrewConstraintInfo::tf_m_to_s
* @brief Start pose
* @var ScrewConstraintInfo::tf_m_to_q
* @brief Forward kinematics of the robot
* @var ScrewConstraintInfo::phi
* @brief Initial guess for the set of screw angles, phi. This is modified in constrainFn
* @var ScrewConstraintInfo::error
* @brief Final error, set by constraintFn
*/
struct ScrewConstraintInfo
{
  //Inputs
  std::vector<ScrewAxis> screw_axis_set;
  const std::pair<Eigen::VectorXd, Eigen::VectorXd> phi_bounds;
  std::queue<Eigen::VectorXd> phi_starts = getGradStarts(phi_bounds);
  const Eigen::Isometry3d tf_m_to_s;
  const Eigen::Isometry3d tf_m_to_q;

  //Outputs
  Eigen::VectorXd phi;
  Eigen::VectorXd error;
  Eigen::VectorXd best_error = Eigen::VectorXd::Constant(6, 1, 1);
  //error is sqrt(6) unless otherwise explicitly determined

private:
  std::queue<Eigen::VectorXd> getGradStarts(
    const std::pair<Eigen::VectorXd, Eigen::VectorXd> & phi_bounds, double max_dist = 0.5 * M_PI);
};

//Functions
/**
* @brief Computes the set of screw angles that minimize the error between EE pose and screw path 
*
* @param tf_m_to_q Forward kinematics of the robot
* @param tf_m_to_s Start pose
* @param screw_axis_set Set of screw axes (S1,...,Sm) in base frame, M
* @param phi_max Set of screw distances to move
* @param phi_start Set of initial random guesses for screw angles
*
* @return Constraint function success, error, and corresponding screw angles
*/
/**
 * @brief Computes the set of screw angles that minimize the error between EE pose and screw path 
 *
 * @param screw_constraint_info struct of constraints
 *
 * @return Constraint function success. Additionally error and corresponding screw angles are set in the screw_constraint_info struct.

 */
bool constraintFn(ScrewConstraintInfo & screw_constraint_info);

/**
 * @brief Computes the partial derivative of error between EE pose and screw path
 *
 * @param tf_q_to_m Inverse of the forward kinematics matrix, tf_m_to_q
 * @param tf_m_to_s Transform from base frame, M to screw path
 * @param phi_current Current set of screw angles
 * @param screw_axis_set Set of screw axes (S1,...,Sm) in base frame, M
 *
 * @return Vector of partial derivative of error with respect of screw angles
 */
Eigen::VectorXd errorDerivative(
  const Eigen::Isometry3d & tf_q_to_m, const Eigen::Isometry3d & tf_m_to_s,
  const Eigen::VectorXd & phi_current, const std::vector<ScrewAxis> & screw_axis_set);

/**
 * @brief Computes the 6x1 error vector for a given transformation matrix
 *
 * @param tf Transformation matrix
 *
 * @return 6x1 Error vector
 */
Eigen::VectorXd calculateEta(const Eigen::Isometry3d & tf);

/**
 * @brief Given a set of screw axes and angles, computes the product of exponentials for specified start and end indices
 *
 * @param screw_axis_set Set of screw axes (S1,...,Sm) in base frame, M
 * @param phi Set of screw angles
 * @param start Start index for the product
 * @param end End index for the product
 *
 * @return Homogenous transformation matrix representing the product of exponentials
 */
Eigen::Isometry3d productOfExponentials(
  const std::vector<ScrewAxis> & screw_axis_set, const Eigen::VectorXd & phi, int start, int end);

/**
 * @brief Clamps the values of array to given highs and lows 
 *
 * @param arr array that is to be clamped
 * @param arr_low array of lows
 * @param arr_high array of highs
 *
 * @return Clamped array 
 */
Eigen::VectorXd clamp(
  const Eigen::VectorXd & arr, const Eigen::VectorXd & arr_low, const Eigen::VectorXd & arr_high);

}  // namespace affordance_primitives
