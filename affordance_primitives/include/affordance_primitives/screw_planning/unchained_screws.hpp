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
#include <affordance_primitives/screw_planning/screw_constraint.hpp>
#include <algorithm>
#include <queue>
#include <utility>
namespace affordance_primitives
{
class UnchainedScrews : public ScrewConstraint
{
public:
  //Variables
  std::pair<Eigen::VectorXd, Eigen::VectorXd> phi_bounds;
  std::queue<Eigen::VectorXd> phi_starts;
  const std::vector<ScrewAxis> & screws;

  //Constructor
  UnchainedScrews(
    const std::vector<ScrewAxis> & screws, const std::vector<double> & lower_bounds,
    const std::vector<double> & upper_bounds, const Eigen::Isometry3d & tf_m_to_s);

  //Mandatory implementations
  virtual bool constraintFn(
    const Eigen::Isometry3d & tf_m_to_q, ScrewConstraintSolution & sol) override;

  virtual Eigen::Isometry3d getPose(const std::vector<double> & phi) const override;

  Eigen::VectorXd errorDerivative(
    const Eigen::Isometry3d & tf_q_to_m, const Eigen::Isometry3d & tf_m_to_s,
    const Eigen::VectorXd & phi_current);

  Eigen::VectorXd calculateEta(const Eigen::Matrix4d & tf);
  Eigen::VectorXd calculateEta(const Eigen::Isometry3d & tf);

  Eigen::Isometry3d productOfExponentials(
    std::vector<ScrewAxis> & screws, const Eigen::VectorXd & phi, int start, int end);

private:
  std::queue<Eigen::VectorXd> getGradStarts(
    const std::pair<Eigen::VectorXd, Eigen::VectorXd> & phi_bounds, double max_dist = 0.5 * M_PI);
};
}  // namespace affordance_primitives
