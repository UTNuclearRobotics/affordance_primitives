///////////////////////////////////////////////////////////////////////////////
//      Title     : screw_constraint.hpp
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

#include <affordance_primitive_msgs/ScrewStamped.h>

#include <affordance_primitives/screw_model/affordance_utils.hpp>
#include <affordance_primitives/screw_model/screw_axis.hpp>
#include <limits>
#include <random>
#include <utility>
#include <vector>

namespace affordance_primitives
{
/**
  * @brief Calculates the error from a TF
  *
  * @param tf The TF. Passing identity should output 0's
  * @return 6x1 vector with [linear ; angular] error
  */
Eigen::VectorXd calculateEta(const Eigen::Matrix4d & tf);
Eigen::VectorXd calculateEta(const Eigen::Isometry3d & tf);

/**
 * @brief The return type for constraint functions
 *
 * Holds a bunch of useful data
 */
struct ScrewConstraintSolution
{
  std::vector<double> solved_phi;
  Eigen::VectorXd error_vector;
  double error = std::numeric_limits<double>::max();

  void reset()
  {
    solved_phi.clear();
    error_vector.setOnes();
    error_vector *= std::numeric_limits<double>::max();
    error = std::numeric_limits<double>::max();
  };
};

/**
 * @brief Manages all operations with screw constrained paths
 *
 * Is used as a base class for various methods of handling multiple screws
 * You must implement constraintFn() AND getPose()
 */
class ScrewConstraint
{
public:
  ScrewConstraint();
  /**
  * @brief Constructors
  *
  * @param screws The vector of screw axes
  * @param start_phi Start screw state, size must match screws size
  * @param goal_phi Goal screw state, size must match screws size
  * @param tf_m_to_s The TF from the affordance (eg map) frame to the path reference frame
  * @param lower_bounds Lower bounds for screws, size must match screws size
  * @param upper_bounds Upper bounds for screws, size must match screws size
  */
  ScrewConstraint(
    const std::vector<ScrewStamped> & screws, const std::vector<double> & start_phi,
    const std::vector<double> & goal_phi, const Eigen::Isometry3d & tf_m_to_s,
    const std::vector<double> & lower_bounds = std::vector<double>(),
    const std::vector<double> & upper_bounds = std::vector<double>());
  ScrewConstraint(
    const std::vector<ScrewAxis> & screws, const std::vector<double> & start_phi,
    const std::vector<double> & goal_phi, const Eigen::Isometry3d & tf_m_to_s,
    const std::vector<double> & lower_bounds = std::vector<double>(),
    const std::vector<double> & upper_bounds = std::vector<double>());

  /**
  * @brief Calculates the constraint function
  * 
  * You must implement this function!
  *
  * @param tf_m_to_q The pose (given in affordance frame) to check for
  * @param phi_0 (Optional) a starting guess for phi
  * @param sol The solution, to be filled in by this function
  * @return True if successful, false if there was a problem
  */
  virtual bool constraintFn(
    const Eigen::Isometry3d & tf_m_to_q, const std::vector<double> & phi_0,
    ScrewConstraintSolution & sol) = 0;
  virtual bool constraintFn(const Eigen::Isometry3d & tf_m_to_q, ScrewConstraintSolution & sol) = 0;

  /**
  * @brief Gets a pose on the path at some state
  * 
  * You must implement this function!
  * 
  * @param phi State vector
  * @return The pose in the affordance frame at state phi
  */
  virtual Eigen::Isometry3d getPose(const std::vector<double> & phi) const;

  /**
  * @brief Sets the path reference frame tf_m_to_s
  */
  virtual void setReferenceFrame(const Eigen::Isometry3d & tf_m_to_s);

  /**
  * @brief Adds a screw axis. Pushes back to the vectors
  */
  virtual void addScrewAxis(const ScrewStamped & axis, double start_theta, double end_theta);
  virtual void addScrewAxis(
    const ScrewStamped & axis, double start_theta, double end_theta, double lower_bound,
    double upper_bound);
  virtual void addScrewAxis(const ScrewAxis & axis, double start_theta, double end_theta);
  virtual void addScrewAxis(
    const ScrewAxis & axis, double start_theta, double end_theta, double lower_bound,
    double upper_bound);

  /**
  * @brief Samples a random valid screw state
  */
  virtual std::vector<double> sampleUniformState() const;

  /**
  * @brief Samples a random valid screw state near another state
  * 
  * @param near the state to sample near
  * @param distance how far away to sample
  */
  virtual std::vector<double> sampleUniformStateNear(
    const std::vector<double> & near, double distance) const;

  /**
  * @brief Samples a random valid screw state via Normal distribution
  * 
  * @param mean the mean state to sample around
  * @param stdDev standard deviation
  */
  virtual std::vector<double> sampleGaussianStateNear(
    const std::vector<double> & mean, double stdDev) const;

  /**
  * @brief Get the screw set for visualizing
  * 
  * Each screw is w.r.t. the affordance (map) frame
  * 
  * @return A vector of screws, all defined in the affordance frame
  */
  virtual std::vector<ScrewStamped> getVisualScrews() const;

  // Getters
  const std::vector<ScrewAxis> & axes() const { return axes_; }
  const Eigen::Isometry3d & referenceFrame() const { return tf_m_to_s_; }
  const std::vector<double> & startPhi() const { return start_phi_; }
  const std::vector<double> & goalPhi() const { return goal_phi_; }
  const std::vector<double> & lowerBounds() const { return lower_bounds_; }
  const std::vector<double> & upperBounds() const { return upper_bounds_; }
  const size_t size() const { return size_; }
  double tolerance() const { return tolerance_; }
  void tolerance(double tol) { tolerance_ = std::move(tol); }

protected:
  std::vector<ScrewAxis> axes_;
  std::vector<double> start_phi_;
  std::vector<double> goal_phi_;
  std::vector<double> lower_bounds_;
  std::vector<double> upper_bounds_;
  Eigen::Isometry3d tf_m_to_s_;
  size_t size_;             // Any inheritors should keep this up to date
  double tolerance_{5e-3};  // TODO tinker with this

  // Random number generation
  std::unique_ptr<std::mt19937> random_gen_;
};

}  // namespace affordance_primitives
