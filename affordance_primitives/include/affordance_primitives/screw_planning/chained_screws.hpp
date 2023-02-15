///////////////////////////////////////////////////////////////////////////////
//      Title     : chained_screws.hpp
//      Project   : affordance_primitives
//      Created   : 02/05/2023
//      Author    : Adam Pettinger and Janak Panthi
//      Copyright : Copyright© The University of Texas at Austin, 2014-2023. All
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

#include <affordance_primitives/screw_model/affordance_utils.hpp>
#include <affordance_primitives/screw_planning/screw_constraint.hpp>
#include <queue>

namespace affordance_primitives
{
/**
 * @brief Constraints class for multiple chained screw axes
 */
class ChainedScrews : public ScrewConstraint
{
public:
  ChainedScrews();
  /**
  * @brief Constructors
  *
  * @param screws The vector of screw axes
  * @param lower_bounds Lower bounds for screws, size must match screws size
  * @param upper_bounds Upper bounds for screws, size must match screws size
  * @param tf_m_to_s The TF from the affordance (eg map) frame to the path reference frame
  */
  ChainedScrews(
    const std::vector<ScrewStamped> & screws, const std::vector<double> & lower_bounds,
    const std::vector<double> & upper_bounds, const Eigen::Isometry3d & tf_m_to_s);
  ChainedScrews(
    const std::vector<ScrewAxis> & screws, const std::vector<double> & lower_bounds,
    const std::vector<double> & upper_bounds, const Eigen::Isometry3d & tf_m_to_s);

  /**
  * @brief Calculates the constraint function
  *
  * @param tf_m_to_q The pose (given in affordance frame) to check for
  * @param phi_0 (Optional) a starting guess for phi
  * @param sol The solution, to be filled in by this function
  * @return True if successful, false if there was a problem
  */
  virtual bool constraintFn(
    const Eigen::Isometry3d & tf_m_to_q, ScrewConstraintSolution & sol) override;
  virtual bool constraintFn(
    const Eigen::Isometry3d & tf_m_to_q, const std::vector<double> & phi_0,
    ScrewConstraintSolution & sol) override;

  /**
  * @brief Gets a pose on the path
  * @param phi State vector
  */
  virtual Eigen::Isometry3d getPose(const std::vector<double> & phi) const override;
  /**
  * @brief Gets a pose on the path
  * @param lambda lambda value
  */
  virtual Eigen::Isometry3d getPose(double lambda) const;

  /**
  * @brief Adds a screw axis, to the end of the chain
  */
  virtual void addScrewAxis(
    const ScrewStamped & axis, double lower_bound, double upper_bound) override;
  virtual void addScrewAxis(
    const ScrewAxis & axis, double lower_bound, double upper_bound) override;

  /**
  * @brief Samples a random valid screw state
  */
  virtual std::vector<double> sampleUniformState() const override;

  /**
  * @brief Samples a random valid screw state near another state
  * 
  * @param near the state to sample near
  * @param distance how far away to sample
  */
  virtual std::vector<double> sampleUniformStateNear(
    const std::vector<double> & near, double distance) const override;

  /**
  * @brief Samples a random valid screw state via Normal distribution
  * 
  * @param mean the mean state to sample around
  * @param stdDev standard deviation
  */
  virtual std::vector<double> sampleGaussianStateNear(
    const std::vector<double> & mean, double stdDev) const override;

  // Getters
  const double lambdaMax() const { return lambda_max_; }
  const double lambdaMin() const { return 0.0; }

  /**
  * @brief Calculates lambda from a state
  * 
  * @param phi The state to check. Must be the correct size, or -1 is returned
  * @param s_index Optional pointer that gets filled with the index of the axis
  * @return lambda
  */
  double getLambda(const std::vector<double> & phi, size_t * s_index = nullptr) const;

  /**
  * @brief Calculates a state from lambda
  * 
  * @param lambda lambda
  * @param s_index Optional pointer that gets filled with the index of the axis
  * @return The state phi
  */
  std::vector<double> getPhi(double lambda, size_t * s_index = nullptr) const;

  /**
  * @brief Get the screw set for visualizing
  * 
  * Each screw is w.r.t. the affordance (map) frame, but translated and rotated by 
  * the motion all the prior screws will have moved the reference frame. If you 
  * visualize these screws, the EE path will align with them as each axis is "activated"
  * 
  * @return A vector of screws, all defined in the affordance frame
  */
  std::vector<ScrewStamped> getVisualScrews() const;

protected:
  double lambda_max_;
};

/**
  * @brief Helper function that calculates gradient descent starts for some constraint
  *
  * @param constraint The constraints to generate for
  * @param grad_starts Starting positions will be added to the queue
  */
void getGradStarts(const ChainedScrews * constraint, std::queue<std::vector<double>> & grad_starts);

/**
  * @brief Helper function for carrying out the constraint function
  *
  * @param constraint The constraints
  * @param tf_m_to_q The pose (given in affordance frame) to check for
  * @param grad_starts Starting positions
  * @param sol The solution, to be filled in by this function
  */
bool chainedConstraintFn(
  const ChainedScrews * constraint, const Eigen::Isometry3d & tf_m_to_q,
  std::queue<std::vector<double>> & grad_starts, ScrewConstraintSolution & sol);

/**
  * @brief Helper function that runs the gradient descent from one start
  *
  * @param constraint The constraints
  * @param phi Starting positions
  * @param tf_m_to_q The pose (given in affordance frame) to check for
  * @param sol The solution, to be filled in by this function
  * @return false if there was a problem, true otherwise (even if not on constraint)
  */
bool runGradDescentOnce(
  const ChainedScrews * constraint, const Eigen::Isometry3d & tf_m_to_q,
  const std::vector<double> & phi0, ScrewConstraintSolution & sol);

/**
  * @brief Calculates the derivative for chained screw axes
  *
  * @param constraint The constraints
  * @param phi Current screw state
  * @param index The index for which screw axis phi is on
  * @return The derivative
  */
double computeDerivativeForIndex(
  const ChainedScrews * constraint, const std::vector<double> & phi, size_t index,
  const Eigen::Isometry3d & tf_q_to_m, const Eigen::Isometry3d & tf_q_to_p);
}  // namespace affordance_primitives
