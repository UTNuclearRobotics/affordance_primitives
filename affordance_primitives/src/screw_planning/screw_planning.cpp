#include <affordance_primitives/screw_planning/screw_planning.hpp>
#include <algorithm>

namespace affordance_primitives
{
Eigen::VectorXd errorDerivative(
  const Eigen::Isometry3d & tf_q_to_m, const Eigen::Isometry3d & tf_m_to_s,
  const Eigen::VectorXd & phi_current, std::vector<ScrewAxis> & screw_axis_set)
{
  //Compute tf_q_to_p
  const int m = screw_axis_set.size();
  const Eigen::Isometry3d pOE = productOfExponentials(screw_axis_set, phi_current, 0, m - 1);
  const Eigen::Isometry3d tf_q_to_p = tf_q_to_m * pOE * tf_m_to_s;

  //Get eta of tf_q_to_p
  const Eigen::VectorXd xi = calculateEta(tf_q_to_p);

  //Construct Xi and Psi matrices
  Eigen::MatrixXd Xi(m, 6 * m);
  Eigen::VectorXd Psi(6 * m, 1);

  //Helper variables declaration
  Eigen::Isometry3d nu_pOE_left;
  Eigen::Isometry3d nu_pOE_right;
  Eigen::Matrix4d nu;
  Eigen::VectorXd jScrewAxis(6, 1);

  for (int j = 0; j < m; j++) {
    //Compute jth row of Xi
    Xi.row(j) << xi.transpose(), Eigen::MatrixXd::Zero(1, m - j * 6);

    //Compute 6x1 segments of Psi
    nu_pOE_left = productOfExponentials(screw_axis_set, phi_current, 0, j);
    nu_pOE_right = productOfExponentials(screw_axis_set, phi_current, j + 1, m - 1);
    nu = tf_q_to_m.matrix() * nu_pOE_left.matrix() *
         screw_axis_set[j].getScrewSkewSymmetricMatrix() * nu_pOE_right.matrix() *
         tf_m_to_s.matrix();
    Psi.segment(6 * j, 6) = calculateEta(nu);
  }

  //Compute and return Lambda
  Eigen::VectorXd Lambda(m, 1);
  Lambda = Xi * Psi;

  return Lambda;
}

bool constraintFn(ScrewConstraintInfo & screw_constraint_info)
{
  //Check if sizes are valid
  if (
    (screw_constraint_info.screw_axis_set.size() != screw_constraint_info.phi.size()) ||
    (screw_constraint_info.screw_axis_set.size() <= 0)) {
    return false;
  }

  //Gradient descent parameters
  const double gamma = 0.05;     //learning parameter
  const size_t nmax = 100;       //max steps
  const double epsilon = 0.001;  //converge limit

  //Redefine references for readability
  const Eigen::Isometry3d & tf_q_to_m = screw_constraint_info.tf_m_to_q.inverse();
  std::vector<ScrewAxis> & screw_axis_set = screw_constraint_info.screw_axis_set;
  const Eigen::Isometry3d & tf_m_to_s = screw_constraint_info.tf_m_to_s;
  Eigen::VectorXd & phi =
    screw_constraint_info.phi;  //initial phi set in the screw_constraint_info struct

  //Retrieve bounds
  const Eigen::VectorXd & phi_min = screw_constraint_info.phi_bounds.first;
  const Eigen::VectorXd & phi_max = screw_constraint_info.phi_bounds.second;
  std::queue<Eigen::VectorXd> & phi_starts = screw_constraint_info.phi_starts;

  //Compute tf_q_to_p
  const int m = screw_constraint_info.screw_axis_set.size();
  Eigen::Isometry3d pOE = productOfExponentials(screw_axis_set, phi, 0, m - 1);
  Eigen::Isometry3d tf_q_to_p = tf_q_to_m * pOE * tf_m_to_s;

  //Compute alpha, the 1/2 squared norm we want to minimize
  double alpha = 0.5 * calculateEta(tf_q_to_p).squaredNorm();

  //Initialize iterating parameters
  double delta = epsilon;
  size_t i = 0;

  while (i < nmax && fabs(delta) >= epsilon) {
    //Compute phi
    phi = phi - gamma * errorDerivative(tf_q_to_m, tf_m_to_s, phi, screw_axis_set);

    //Clamp phi between phi_min and phi_max
    auto eigen_clamp = [&phi](const Eigen::VectorXd & phi_min, const Eigen::VectorXd & phi_max) {
      for (int i = 0; i < phi.size(); i++) phi[i] = std::clamp(phi[i], phi_min[i], phi_max[i]);
      return phi;
    };
    eigen_clamp(phi_min, phi_max);

    //Compute tf_q_to_p
    pOE = productOfExponentials(screw_axis_set, phi, 0, m - 1);
    tf_q_to_p = tf_q_to_m * pOE * tf_m_to_s;

    //Compute new delta
    delta = alpha - 0.5 * calculateEta(tf_q_to_p).squaredNorm();

    //Store last squared norm as alpha
    alpha = 0.5 * calculateEta(tf_q_to_p).squaredNorm();

    //Increment iteration
    i++;
  }

  //Compute error and update best_error
  screw_constraint_info.error = calculateEta(tf_q_to_p);

  if (screw_constraint_info.error.norm() < screw_constraint_info.best_error.norm())
    screw_constraint_info.best_error = screw_constraint_info.error;

  //Recursively call this function with a new guess for phi until the following conditions are met
  if (!phi_starts.empty() && (screw_constraint_info.best_error.norm() > 5e-3)) {
    phi = phi_starts.front();
    phi_starts.pop();
    return constraintFn(screw_constraint_info);
  } else
    return true;
}

bool chainedConstraintFn(ScrewConstraintInfo & sci)
{
  //Check if sizes are valid
  if ((sci.screw_axis_set.size() != sci.phi.size()) || (sci.screw_axis_set.size() <= 0)) {
    return false;
  }

  //Gradient descent parameters
  const double gamma = 0.05;     //learning parameter
  const size_t nmax = 100;       //max steps
  const double epsilon = 0.001;  //converge limit

  //Redefine references for readability
  const Eigen::Isometry3d tf_q_to_m = sci.tf_m_to_q.inverse();

  //Retrieve bounds
  const Eigen::VectorXd & phi_min = sci.phi_bounds.first;
  const Eigen::VectorXd & phi_max = sci.phi_bounds.second;
  const double lambda_max = getLambda(phi_max, sci.phi_bounds);

  //Compute tf_q_to_p
  const int m = sci.screw_axis_set.size();
  Eigen::Isometry3d pOE = productOfExponentials(sci.screw_axis_set, sci.phi, 0, m - 1);
  Eigen::Isometry3d tf_q_to_p = tf_q_to_m * pOE * sci.tf_m_to_s;

  //Compute alpha, the 1/2 squared norm we want to minimize
  double alpha = 0.5 * calculateEta(tf_q_to_p).squaredNorm();

  //Initialize iterating parameters
  double delta = epsilon;
  size_t i = 0;

  while (i < nmax && fabs(delta) >= epsilon) {
    // Find lambda for this state
    size_t s_index;
    double lambda = getLambda(sci.phi, sci.phi_bounds, &s_index);

    // Calculate the derivative
    auto deriv = computeDerivativeForIndex(
      sci.screw_axis_set, s_index, sci.phi, tf_q_to_m, sci.tf_m_to_s, tf_q_to_p);

    // Update lambda
    lambda -= gamma * deriv;

    // Clamp then get phi
    lambda = std::max(std::min(lambda_max, lambda), 0.0);
    sci.phi = getPhi(lambda, sci.phi_bounds);

    // Compute tf_q_to_p
    pOE = productOfExponentials(sci.screw_axis_set, sci.phi, 0, m - 1);
    tf_q_to_p = tf_q_to_m * pOE * sci.tf_m_to_s;

    // Compute new delta
    const auto new_alpha = 0.5 * calculateEta(tf_q_to_p).squaredNorm();
    delta = alpha - new_alpha;

    // Store last squared norm as alpha
    alpha = new_alpha;
    i++;
  }

  //Compute error and update best_error
  sci.error = calculateEta(tf_q_to_p);

  if (sci.error.norm() < sci.best_error.norm()) sci.best_error = sci.error;

  //Recursively call this function with a new guess for phi until the following conditions are met
  if (!sci.phi_starts.empty() && (sci.best_error.norm() > 5e-3)) {
    sci.phi = sci.phi_starts.front();
    sci.phi_starts.pop();
    return chainedConstraintFn(sci);
  } else
    return true;
}

double computeDerivativeForIndex(
  const std::vector<affordance_primitives::ScrewAxis> & axes, size_t index,
  const Eigen::VectorXd & phi, const Eigen::Isometry3d & tf_q_to_m,
  const Eigen::Isometry3d & tf_m_to_s, const Eigen::Isometry3d & tf_q_to_p)
{
  auto nu_pOE_left = productOfExponentials(axes, phi, 0, index);
  auto nu_pOE_right = productOfExponentials(axes, phi, index + 1, axes.size() - 1);
  auto nu = tf_q_to_m.matrix() * nu_pOE_left.matrix() * axes[index].getScrewSkewSymmetricMatrix() *
            nu_pOE_right.matrix() * tf_m_to_s.matrix();

  return calculateEta(tf_q_to_p).dot(calculateEta(nu));
}

Eigen::Isometry3d productOfExponentials(
  const std::vector<ScrewAxis> & screw_axis_set, const Eigen::VectorXd & phi, int start, int end)
{
  //recursively compute product of exponentials until the POE size is reduced to 1
  if (start < end)
    return screw_axis_set[start].getTF(phi[start]) *
           productOfExponentials(screw_axis_set, phi, start + 1, end);

  //when size is 1 return identity
  else
    return screw_axis_set[0].getTF(0.0);
}

Eigen::VectorXd calculateEta(const Eigen::Matrix4d & tf)
{
  //variable declaration
  const Eigen::AngleAxisd axisAngleRep(tf.block<3, 3>(0, 0));
  const Eigen::Vector3d lin = tf.block<3, 1>(0, 3);
  Eigen::VectorXd eta(6);

  //First three elements denote the translation part
  eta.head(3) = lin;

  //Last three for axis*angle
  eta.tail(3) = axisAngleRep.angle() * axisAngleRep.axis();

  return eta;
}

Eigen::VectorXd calculateEta(const Eigen::Isometry3d & tf)
{
  //variable declaration
  const Eigen::AngleAxisd axisAngleRep(tf.rotation());
  const Eigen::Vector3d lin = tf.translation();
  Eigen::VectorXd eta(6);

  //First three elements denote the translation part
  eta.head(3) = lin;

  //Last three for axis*angle
  eta.tail(3) = axisAngleRep.angle() * axisAngleRep.axis();

  return eta;
}

std::queue<Eigen::VectorXd> ScrewConstraintInfo::getGradStarts(
  const std::pair<Eigen::VectorXd, Eigen::VectorXd> & phi_bounds, double max_dist)
{
  std::queue<Eigen::VectorXd> output;
  const size_t & screw_set_size = phi_bounds.first.size();
  Eigen::MatrixXd
    cond_grad_starts;  //grad descent starts in condensed form with no regard to series constraint
  Eigen::MatrixXd
    dist_grad_starts;  //grad descent starts in distributed form accounting for series constraint

  const Eigen::VectorXd span = (phi_bounds.second - phi_bounds.first).cwiseAbs();
  const size_t num_starts =
    ceil(span.maxCoeff() / max_dist) + 1;  //use the max span to determine number of starts
  const Eigen::VectorXd real_step = span / num_starts;

  for (size_t i = 0; i <= num_starts; ++i) {
    cond_grad_starts.col(i) = phi_bounds.first + i * real_step;
  }

  //reshape to distributed form
  for (int i = 0; i < screw_set_size; i++) {
    //fill mins and active screw sample
    dist_grad_starts.row(i) << Eigen::MatrixXd::Constant(
      1, (num_starts * i), cond_grad_starts.coeff(i, 0)),
      cond_grad_starts.row(i);
    for (int j = 0; j <= i; j++) {
      //fill max elements
      dist_grad_starts.row(i) << Eigen::MatrixXd::Constant(
        1, num_starts * (screw_set_size - 1 - i),
        cond_grad_starts.coeff(j, cond_grad_starts.row(0).size() - 1));
    }
  }

  // output is a queue of columns of dist_grad_starts
  for (int i = 0; i < dist_grad_starts.col(0).size(); i++) output.push(dist_grad_starts.col(i));

  return output;
}

double getLambda(
  const Eigen::VectorXd & phi, const std::pair<Eigen::VectorXd, Eigen::VectorXd> & phi_bounds,
  size_t * s_index)
{
  double lambda = 0;
  size_t s = 0;
  const double EPSILON = 1e-6;

  for (s = 0; s < phi.size(); ++s) {
    if (fabs(phi(s) - phi_bounds.first(s)) < EPSILON) {
      if (s_index) {
        *s_index = std::max(size_t(0), s - 1);
      }
      return lambda;
    } else {
      lambda += phi(s) - phi_bounds.first(s);
    }
  }
  if (s_index) {
    *s_index = s;
  }
  return lambda;
}

Eigen::VectorXd getPhi(
  const double lambda, const std::pair<Eigen::VectorXd, Eigen::VectorXd> & phi_bounds,
  size_t * s_index)
{
  Eigen::VectorXd phi = phi_bounds.first;
  size_t s = 0;
  double b_now = 0;
  double b_next = phi_bounds.second(0) - phi_bounds.first(0);

  for (size_t i = 0; i < phi.size(); ++i) {
    if (lambda < b_now) {
      phi(i) = phi_bounds.first(i);
    } else if (lambda > b_next) {
      phi(i) = phi_bounds.second(i);
    } else {
      phi(i) = phi_bounds.first(i) + lambda - b_now;
      s = i;
    }

    if ((i + 1) < phi.size()) {
      b_now = b_next;
      b_next += phi_bounds.second(i + 1) - phi_bounds.first(i + 1);
    }
  }
  if (s_index) {
    *s_index = s;
  }
  return phi;
}

}  // namespace affordance_primitives
