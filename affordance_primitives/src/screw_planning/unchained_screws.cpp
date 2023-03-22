#include <affordance_primitives/screw_planning/unchained_screws.hpp>
namespace affordance_primitives
{
bool recursiveSearch(
  const UnchainedScrews * constraint, const Eigen::Isometry3d & tf_m_to_q,
  std::queue<Eigen::VectorXd> & phi_starts_, ScrewConstraintSolution & sol)
{
  Eigen::VectorXd phi = phi_starts_.front();

  //Check if sizes are valid
  if ((phi.size() != constraint->size())) {
    return false;
  }

  //Gradient descent parameters
  const double gamma = 0.1;        //learning parameter
  const size_t nmax = 100;         //max steps
  const double epsilon = 0.00001;  //converge limit

  //Helper parameters
  const Eigen::Isometry3d tf_q_to_m = tf_m_to_q.inverse();

  //Compute tf_q_to_p
  Eigen::Isometry3d pOE = productOfExponentials(constraint->axes(), phi, 0, constraint->size() - 1);
  Eigen::Isometry3d tf_q_to_p = tf_q_to_m * pOE * constraint->referenceFrame();

  //Compute alpha, the 1/2 squared norm we want to minimize
  double alpha = 0.5 * calculateEta(tf_q_to_p).squaredNorm();

  //Initialize iterating parameters
  double delta = epsilon;
  size_t i = 0;

  /* To clamp phi in the loop */
  auto eigen_clamp = [&phi](
                       const std::vector<double> & phi_min, const std::vector<double> & phi_max) {
    for (int i = 0; i < phi_min.size(); i++) phi[i] = std::clamp(phi[i], phi_min[i], phi_max[i]);
    return phi;
  };

  while (i < nmax && fabs(delta) >= epsilon) {
    //Compute phi
    const auto deriv =
      errorDerivative(tf_q_to_m, constraint->referenceFrame(), phi, constraint->axes());
    phi = phi -
          gamma * errorDerivative(tf_q_to_m, constraint->referenceFrame(), phi, constraint->axes());

    eigen_clamp(constraint->lowerBounds(), constraint->upperBounds());

    //Compute tf_q_to_p
    pOE = productOfExponentials(constraint->axes(), phi, 0, constraint->size() - 1);
    tf_q_to_p = tf_q_to_m * pOE * constraint->referenceFrame();

    //Compute new delta
    delta = alpha - 0.5 * calculateEta(tf_q_to_p).squaredNorm();

    //Store last squared norm as alpha
    alpha = 0.5 * calculateEta(tf_q_to_p).squaredNorm();

    //Increment iteration
    i++;
  }

  //Compute error and update best_error and corresponding solved_phi
  const auto current_error = calculateEta(tf_q_to_p);

  if (current_error.norm() < sol.error) {
    sol.error_vector = current_error;
    sol.error = current_error.norm();
    std::vector<double> solved_phi_svec(phi.data(), phi.data() + phi.size());  // Temporary, TODO
    sol.solved_phi = solved_phi_svec;
  }

  //Pop last phi guess and recursively call this function with a new guess until all guesses are exhausted
  phi_starts_.pop();
  if (!phi_starts_.empty()) {
    return recursiveSearch(constraint, tf_m_to_q, phi_starts_, sol);
  } else {
    return true;
  }
}

Eigen::Isometry3d productOfExponentials(
  const std::vector<ScrewAxis> & screws, const Eigen::VectorXd & phi, size_t start, size_t end)
{
  //recursively compute product of exponentials until the POE size is reduced to 1
  if (start <= end)
    return screws[start].getTF(phi[start]) * productOfExponentials(screws, phi, start + 1, end);

  //when size is 1 return identity
  else
    return screws[0].getTF(0.0);
}

Eigen::VectorXd errorDerivative(
  const Eigen::Isometry3d & tf_q_to_m, const Eigen::Isometry3d & tf_m_to_s,
  const Eigen::VectorXd & phi_current, const std::vector<ScrewAxis> & screws)
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

UnchainedScrews::UnchainedScrews() : ScrewConstraint(){};
UnchainedScrews::UnchainedScrews(
  const std::vector<ScrewAxis> & screws, const std::vector<double> & lower_bounds,
  const std::vector<double> & upper_bounds, const Eigen::Isometry3d & tf_m_to_s)
: ScrewConstraint(screws, lower_bounds, upper_bounds, tf_m_to_s)
{
  phi_starts_ = getGradStarts(lower_bounds_, upper_bounds_);
}

UnchainedScrews::UnchainedScrews(
  const std::vector<ScrewStamped> & screws, const std::vector<double> & lower_bounds,
  const std::vector<double> & upper_bounds, const Eigen::Isometry3d & tf_m_to_s)
: ScrewConstraint(screws, lower_bounds, upper_bounds, tf_m_to_s)
{
  phi_starts_ = getGradStarts(lower_bounds_, upper_bounds_);
}

bool UnchainedScrews::constraintFn(
  const Eigen::Isometry3d & tf_m_to_q, ScrewConstraintSolution & sol)
{
  if (size_ < 1) {
    return false;
  }

  sol.reset();

  auto starts = phi_starts_;

  return recursiveSearch(this, tf_m_to_q, starts, sol);
}

bool UnchainedScrews::constraintFn(
  const Eigen::Isometry3d & tf_m_to_q, const std::vector<double> & phi_0,
  ScrewConstraintSolution & sol)
{
  if (size_ < 1 || phi_0.size() != size_) {
    return false;
  }

  sol.reset();

  // Add the passed state to phi_starts
  Eigen::VectorXd this_start(size_);
  for (size_t i = 0; i < size_; ++i) {
    this_start(i) = phi_0.at(i);
  }
  auto saved_starts = phi_starts_;
  std::queue<Eigen::VectorXd> starts;
  starts.push(this_start);
  while (!saved_starts.empty()) {
    starts.push(saved_starts.front());
    saved_starts.pop();
  }

  return recursiveSearch(this, tf_m_to_q, starts, sol);
}

std::queue<Eigen::VectorXd> UnchainedScrews::getGradStarts(
  const std::vector<double> & lower_bounds, const std::vector<double> & upper_bounds,
  double max_dist)
{
  std::queue<Eigen::VectorXd> output;

  Eigen::VectorXd bounds_high(upper_bounds.size());
  Eigen::VectorXd bounds_low(lower_bounds.size());
  for (size_t i = 0; i < upper_bounds.size(); ++i) {
    bounds_high(i) = upper_bounds.at(i);
    bounds_low(i) = lower_bounds.at(i);
  }

  const Eigen::VectorXd span = (bounds_high - bounds_low).cwiseAbs();
  const size_t num_starts =
    ceil(span.maxCoeff() / max_dist) + 1;  //use the max span to determine number of starts
  const Eigen::VectorXd real_step = span / num_starts;

  Eigen::MatrixXd cond_grad_starts(
    size_,
    num_starts + 1);  //grad descent starts in condensed form with no regard to series constraint
  Eigen::MatrixXd dist_grad_starts(
    size_,
    size_ * (num_starts) +
      1);  //grad descent starts in distributed form accounting for series constraint

  for (size_t i = 0; i <= num_starts; ++i) {
    cond_grad_starts.col(i) = bounds_low + i * real_step;
  }

  for (int i = 0; i < cond_grad_starts.row(0).size(); i++) {
    output.push(cond_grad_starts.col(i));
  }
  // bool chained = false;

  // if (chained) {  //temporary conditional for testing
  //   //reshape to distributed form
  //   for (int i = 0; i < size_; i++) {
  //     dist_grad_starts.row(i) << Eigen::RowVectorXd::Constant(
  //       (num_starts * i), cond_grad_starts.coeff(i, 0)),
  //       cond_grad_starts.row(i).head(cond_grad_starts.row(i).size() - 1),
  //       Eigen::RowVectorXd::Constant(
  //         num_starts * (size_ - 1 - i) + 1,
  //         cond_grad_starts.coeff(i, cond_grad_starts.row(0).size() - 1));
  //   }

  //   // output is a queue of columns of dist_grad_starts
  //   for (int i = 0; i < dist_grad_starts.row(0).size(); i++) output.push(dist_grad_starts.col(i));
  // }

  // else {
  //   for (int i = 0; i < cond_grad_starts.row(0).size(); i++) output.push(cond_grad_starts.col(i));
  // }

  return output;
}

void UnchainedScrews::addScrewAxis(const ScrewStamped & axis, double start_theta, double end_theta)
{
  ScrewConstraint::addScrewAxis(axis, start_theta, end_theta);
  phi_starts_ = getGradStarts(lower_bounds_, upper_bounds_);
}

void UnchainedScrews::addScrewAxis(
  const ScrewStamped & axis, double start_theta, double end_theta, double lower_bound,
  double upper_bound)
{
  ScrewConstraint::addScrewAxis(axis, start_theta, end_theta, lower_bound, upper_bound);
  phi_starts_ = getGradStarts(lower_bounds_, upper_bounds_);
}

void UnchainedScrews::addScrewAxis(const ScrewAxis & axis, double start_theta, double end_theta)
{
  ScrewConstraint::addScrewAxis(axis, start_theta, end_theta);
  phi_starts_ = getGradStarts(lower_bounds_, upper_bounds_);
}

void UnchainedScrews::addScrewAxis(
  const ScrewAxis & axis, double start_theta, double end_theta, double lower_bound,
  double upper_bound)
{
  ScrewConstraint::addScrewAxis(axis, start_theta, end_theta, lower_bound, upper_bound);
  phi_starts_ = getGradStarts(lower_bounds_, upper_bounds_);
}

}  // namespace affordance_primitives
