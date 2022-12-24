///////////////////////////////////////////////////////////////////////////////
//      Title     : screw_planning.hpp
//      Project   : affordance_primitives
//      Created   : 12/24/2022
//      Author    : Janak Panthi
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

#include <affordance_primitives/screw_planning/screw_planning.hpp>
namespace affordance_primitives
{
Eigen::VectorXd errorDerivative(
  const Eigen::Isometry3d & tf_q_to_m, const Eigen::Isometry3d & tf_m_to_s,
  const Eigen::VectorXd & phi_current, const std::vector<ScrewAxis> & screw_axis_set)
{
  //Compute tf_q_to_p
  int m = screw_axis_set.size();
  const Eigen::Isometry3d pOE = productOfExponentials(screw_axis_set, phi_current, 0, m - 1);
  const Eigen::Isometry3d tf_q_to_p = tf_q_to_m * pOE * tf_m_to_s;

  //Get eta of tf_q_to_p
  const Eigen::VectorXd xi = eta(tf_q_to_p);

  //Construct Xi and Psi matrices
  Eigen::MatrixXd Xi(m, 6 * m);
  Eigen::VectorXd Psi(6 * m, 1);

  //Helper variables declaration
  Eigen::Isometry3d nu_pOE_left;
  Eigen::Isometry3d nu_pOE_right;
  Eigen::Isometry3d nu;
  Eigen::VectorXd jScrewAxis(6, 1);

  for (int j = 0; j < m; j++) {
    //Compute jth row of Xi
    Xi.row(j) << xi.transpose(), Eigen::MatrixXd::Zero(1, m - j * 6);

    //Compute 6x1 segments of Psi
    nu_pOE_left = productOfExponentials(screw_axis_set, phi_current, 0, j);
    nu_pOE_right = productOfExponentials(screw_axis_set, phi_current, j + 1, m - 1);
    jScrewAxis << screw_axis_set[j].getAxis(), screw_axis_set[j].getLinearVector();
    nu = tf_q_to_m * nu_pOE_left * affordance_primitives::getSkewSymmetricMatrix(jScrewAxis) *
         nu_pOE_right * tf_m_to_s;
    Psi.segment(6 * j, 6) = eta(nu);
  }

  //Compute and return Lambda
  Eigen::VectorXd Lambda(m, 1);
  Lambda = Xi * Psi;

  return Lambda;
}

std::tuple<bool, Eigen::VectorXd, Eigen::VectorXd> constraintFn(
  const Eigen::Isometry3d & tf_m_to_q, const Eigen::Isometry3d & tf_m_to_s,
  const std::vector<ScrewAxis> & screw_axis_set, const Eigen::VectorXd phi_max,
  const Eigen::VectorXd phi_start)
{
  //Gradient descent parameters
  const double gamma = 0.05;     //learning parameter
  const size_t nmax = 100;       //max steps
  const double epsilon = 0.001;  //converge limit

  //Compute tf_q_to_m
  const Eigen::Isometry3d tf_q_to_m = tf_m_to_q.inverse();

  //Sample random phi
  Eigen::VectorXd phi = phi_start;  //phi_start is already randomly sampled and fed to this function

  //Compute tf_q_to_p
  int m = screw_axis_set.size();
  Eigen::Isometry3d pOE = productOfExponentials(screw_axis_set, phi_start, 0, m - 1);
  Eigen::Isometry3d tf_q_to_p = tf_m_to_q.inverse() * pOE * tf_m_to_s;

  //Compute alpha, the 1/2 squared norm we want to minimize
  double alpha = 0.5 * eta(tf_q_to_p).squaredNorm();

  //Initialize iterating parameters
  double delta = epsilon;
  size_t i = 0;

  while (i < nmax && fabs(delta) >= epsilon) {
    //Compute phi
    phi = phi - gamma * errorDerivative(tf_q_to_m, tf_m_to_s, phi, screw_axis_set);

    //Clamp phi between zeros and phi_max
    phi = clamp(phi, Eigen::VectorXd::Zero(6, 1), phi_max);

    //Compute tf_q_to_p
    pOE = productOfExponentials(screw_axis_set, phi, 0, m - 1);
    tf_q_to_p = tf_m_to_q.inverse() * pOE * tf_m_to_s;

    //Compute new delta
    delta = alpha - 0.5 * eta(tf_q_to_p).squaredNorm();

    //store last squared norm as alpha
    alpha = 0.5 * eta(tf_q_to_p).squaredNorm();

    //increment iteration
    i++;
  }

  //return true, final error, and corresponding phi
  return std::make_tuple(true, eta(tf_q_to_p), phi);
}

Eigen::Isometry3d productOfExponentials(
  const std::vector<ScrewAxis> & screw_axis_set, const Eigen::VectorXd & phi, int start, int end)
{
  //assert screw_axis_set ref is not null and size is positive and non-zero
  assert(&screw_axis_set != NULL && screw_Axis_set.size() > 0 && phi.size() > 0);

  //recursively compute product of exponentials until the POE size is reduced to 1
  if (start < end)
    return screw_axis_set[start].getTF(phi[start]) *
           productOfExponentials(screw_axis_set, phi, start + 1, end);

  //when size is 1 return identity
  else
    return screw_axis_set[0].getTF(0.0);
}

Eigen::VectorXd eta(const Eigen::Isometry3d & tf)
{
  //variable declaration
  const Eigen::AngleAxisd axisAngleRep(tf.rotation());
  const Eigen::Vector3d lin = tf.linear();
  Eigen::VectorXd eta(6);

  //First three elements denote the translation part
  eta.head(3) = lin;

  //Last three for axis*angle
  eta.tail(3) = axisAngleRep.angle() * axisAngleRep.axis();

  return eta;
}

Eigen::VectorXd clamp(
  const Eigen::VectorXd arr, const Eigen::VectorXd arr_low, const Eigen::VectorXd arr_high)
{
  //Declare array of same size to return
  Eigen::VectorXd arr_clamped(arr.size());

  //clamp and return
  for (int i = 0; i < arr.size(); i++) arr_clamped[i] = std::clamp(arr[i], arr_low[i], arr_high[i]);
  return arr_clamped;
}
}  // namespace affordance_primitives
