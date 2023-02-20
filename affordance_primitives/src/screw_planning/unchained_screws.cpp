#include <affordance_primitives/screw_planning/unchained_screws.hpp>
namespace affordance_primitives
{
UnchainedScrews::UnchainedScrews(
  const std::vector<ScrewAxis> & screws, const std::vector<double> & lower_bounds,
  const std::vector<double> & upper_bounds, const Eigen::Isometry3d & tf_m_to_s)
: ScrewConstraint(screws, lower_bounds, upper_bounds, tf_m_to_s)
{
  //Temporary, TODO
  Eigen::VectorXd phi_low = Eigen::VectorXd::Map(lower_bounds.data(), lower_bounds.size());
  Eigen::VectorXd phi_high = Eigen::VectorXd::Map(upper_bounds.data(), upper_bounds.size());
  phi_bounds = {phi_low, phi_high};

  phi_starts = getGradStarts(phi_bounds);
}
Eigen::VectorXd UnchainedScrews::errorDerivative(
  const Eigen::Isometry3d & tf_q_to_m, const Eigen::Isometry3d & tf_m_to_s,
  const Eigen::VectorXd & phi_current, std::vector<ScrewAxis> & screws)
{
  //Compute tf_q_to_p
  const int m = screws.size();  //TODO
  const Eigen::Isometry3d pOE = productOfExponentials(screws, phi_current, 0, m - 1);
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
    Xi.row(j) << Eigen::MatrixXd::Zero(1, j * 6), xi.transpose(),
      Eigen::MatrixXd::Zero(1, (m - 1 - j) * 6);

    //Compute 6x1 segments of Psi
    nu_pOE_left = productOfExponentials(screws, phi_current, 0, j);
    nu_pOE_right = productOfExponentials(screws, phi_current, j + 1, m - 1);
    nu = tf_q_to_m.matrix() * nu_pOE_left.matrix() * screws[j].getScrewSkewSymmetricMatrix() *
         nu_pOE_right.matrix() * tf_m_to_s.matrix();
    Psi.segment(6 * j, 6) = calculateEta(nu);
  }

  //Compute and return Lambda
  Eigen::VectorXd Lambda(m, 1);
  Lambda = Xi * Psi;

  return Lambda;
}

bool UnchainedScrews::constraintFn(ScrewConstraintInfo & sol)
{
  Eigen::VectorXd & phi = phi_starts.front();

  //Check if sizes are valid, TODO: Might not need to implement this. Base class has it?
  if ((screws.size() != phi.size()) || (screws.size() <= 0)) {
    return false;
  }

  //Gradient descent parameters
  const double gamma = 0.05;     //learning parameter
  const size_t nmax = 100;       //max steps
  const double epsilon = 0.001;  //converge limit

  //Helper parameters
  const Eigen::Isometry3d & tf_q_to_m = tf_m_to_q.inverse();

  //Retrieve bounds
  const Eigen::VectorXd & phi_min = phi_bounds.first;
  const Eigen::VectorXd & phi_max = phi_bounds.second;

  //Compute tf_q_to_p
  const int m = screws.size();  //TODO
  Eigen::Isometry3d pOE = productOfExponentials(screws, phi, 0, m - 1);
  Eigen::Isometry3d tf_q_to_p = tf_q_to_m * pOE * tf_m_to_s;

  //Compute alpha, the 1/2 squared norm we want to minimize
  double alpha = 0.5 * calculateEta(tf_q_to_p).squaredNorm();

  //Initialize iterating parameters
  double delta = epsilon;
  size_t i = 0;

  //To clamp phi in the loop
  auto eigen_clamp = [&phi](const Eigen::VectorXd & phi_min, const Eigen::VectorXd & phi_max) {
    for (int i = 0; i < phi.size(); i++) phi[i] = std::clamp(phi[i], phi_min[i], phi_max[i]);
    return phi;
  };

  while (i < nmax && fabs(delta) >= epsilon) {
    //Compute phi
    phi = phi - gamma * errorDerivative(tf_q_to_m, tf_m_to_s, phi, screws);

    eigen_clamp(phi_min, phi_max);

    //Compute tf_q_to_p
    pOE = productOfExponentials(screws, phi, 0, m - 1);
    tf_q_to_p = tf_q_to_m * pOE * tf_m_to_s;

    //Compute new delta
    delta = alpha - 0.5 * calculateEta(tf_q_to_p).squaredNorm();

    //Store last squared norm as alpha
    alpha = 0.5 * calculateEta(tf_q_to_p).squaredNorm();

    //Increment iteration
    i++;
  }

  //Compute error and update best_error and corresponding solved_phi
  current_error = calculateEta(tf_q_to_p);

  if (current_error.norm() < sol.error) {
    sol.error_vector = current_error;
    sol.error = current_error.norm();
    std::vector<double> solved_phi_svec(phi.data(), phi.data() + phi.size());  // Temporary, TODO
    sol.solved_phi = solved_phi_svec;
  }

  //Pop last phi guess and recursively call this function with a new guess until all guesses are exhausted
  phi_starts.pop();
  if (!phi_starts.empty()) {
    return constraintFn(sol);
  } else {
    return true;
  }
}

Eigen::Isometry3d UnchainedScrews::productOfExponentials(
  std::vector<ScrewAxis> & screws, const Eigen::VectorXd & phi, int start, int end)
{
  //recursively compute product of exponentials until the POE size is reduced to 1
  if (start < end)
    return screws[start].getTF(phi[start]) * productOfExponentials(screws, phi, start + 1, end);

  //when size is 1 return identity
  else
    return screws[0].getTF(0.0);
}

Eigen::VectorXd UnchainedScrews::calculateEta(const Eigen::Matrix4d & tf)
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

Eigen::VectorXd UnchainedScrews::calculateEta(const Eigen::Isometry3d & tf)
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

  const Eigen::VectorXd span = (phi_bounds.second - phi_bounds.first).cwiseAbs();
  const size_t num_starts =
    ceil(span.maxCoeff() / max_dist) + 1;  //use the max span to determine number of starts
  const Eigen::VectorXd real_step = span / num_starts;

  Eigen::MatrixXd cond_grad_starts(
    screw_set_size,
    num_starts + 1);  //grad descent starts in condensed form with no regard to series constraint
  Eigen::MatrixXd dist_grad_starts(
    screw_set_size,
    screw_set_size * (num_starts) +
      1);  //grad descent starts in distributed form accounting for series constraint

  for (size_t i = 0; i <= num_starts; ++i) {
    cond_grad_starts.col(i) = phi_bounds.first + i * real_step;
  }

  bool chained = false;

  if (chained) {  //temporary conditional for testing
    //reshape to distributed form
    for (int i = 0; i < screw_set_size; i++) {
      dist_grad_starts.row(i) << Eigen::RowVectorXd::Constant(
        (num_starts * i), cond_grad_starts.coeff(i, 0)),
        cond_grad_starts.row(i).head(cond_grad_starts.row(i).size() - 1),
        Eigen::RowVectorXd::Constant(
          num_starts * (screw_set_size - 1 - i) + 1,
          cond_grad_starts.coeff(i, cond_grad_starts.row(0).size() - 1));
    }

    // output is a queue of columns of dist_grad_starts
    for (int i = 0; i < dist_grad_starts.row(0).size(); i++) output.push(dist_grad_starts.col(i));
  }

  else {
    for (int i = 0; i < cond_grad_starts.row(0).size(); i++) output.push(cond_grad_starts.col(i));
  }

  return output;
}

// TODO, This function is unused at the moment
Eigen::Isometry3d UnchainedScrews::getPose(const std::vector<double> & phi) const
{
  Eigen::Isometry3d output = Eigen::Isometry3d::Identity();

  // Check input
  if (phi.size() != size_) {
    return output;
  }

  // Step through each axis and calculate
  for (size_t i = 0; i < size_; ++i) {
    output = output * axes_.at(i).getTF(phi[i]);
  }
  output = output * tf_m_to_s_;
  return output;
}  // namespace affordance_primitives
