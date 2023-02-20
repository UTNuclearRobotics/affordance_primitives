#include <affordance_primitives/screw_planning/unchained_screws.hpp>
namespace affordance_primitives
{
bool recursiveSearch(
  const UnchainedScrews * constraint, const Eigen::Isometry3d & tf_m_to_q,
  std::queue<Eigen::VectorXd> & phi_starts, ScrewConstraintSolution & sol)
{
  Eigen::VectorXd phi = phi_starts.front();

  //Check if sizes are valid
  if ((phi.size() != constraint->size())) {
    return false;
  }

  //Gradient descent parameters
  const double gamma = 0.1;       //learning parameter
  const size_t nmax = 100;        //max steps
  const double epsilon = 0.0001;  //converge limit

  //Helper parameters
  const Eigen::Isometry3d tf_q_to_m = tf_m_to_q.inverse();

  //Retrieve bounds
  // const Eigen::VectorXd & phi_min = phi_bounds.first;
  // const Eigen::VectorXd & phi_max = phi_bounds.second;

  //Compute tf_q_to_p
  Eigen::Isometry3d pOE = productOfExponentials(constraint->axes(), phi, 0, constraint->size() - 1);
  Eigen::Isometry3d tf_q_to_p = tf_q_to_m * pOE * constraint->referenceFrame();

  //Compute alpha, the 1/2 squared norm we want to minimize
  double alpha = 0.5 * calculateEta(tf_q_to_p).squaredNorm();

  //Initialize iterating parameters
  double delta = std::numeric_limits<double>::max();
  size_t i = 0;

  //To clamp phi in the loop
  // auto eigen_clamp = [&phi](
  //                      const std::vector<double> & phi_min, const std::vector<double> & phi_max) {
  //   for (int i = 0; i < phi_min.size(); i++) phi[i] = std::clamp(phi[i], phi_min[i], phi_max[i]);
  //   return phi;
  // };

  while (i < nmax && fabs(delta) >= epsilon) {
    //Compute phi
    std::cout << "i = " << i << "\nphi before = " << phi.transpose();
    const auto deriv =
      errorDerivative(tf_q_to_m, constraint->referenceFrame(), phi, constraint->axes());
    phi = phi -
          gamma * errorDerivative(tf_q_to_m, constraint->referenceFrame(), phi, constraint->axes());

    std::cout << "\nphi mid = " << phi.transpose();
    // eigen_clamp(constraint->lowerBounds(), constraint->upperBounds());
    for (size_t j = 0; j < phi.size(); j++) {
      phi[j] = std::clamp(phi[j], constraint->lowerBounds().at(j), constraint->upperBounds().at(j));
    }
    std::cout << "\nderivative = " << deriv.transpose() << "\nphi after = " << phi.transpose()
              << "\n";

    //Compute tf_q_to_p
    pOE = productOfExponentials(constraint->axes(), phi, 0, constraint->size() - 1);
    tf_q_to_p = tf_q_to_m * pOE * constraint->referenceFrame();

    //Compute new delta
    std::cout << "delta_before = " << delta << "\n";
    delta = alpha - 0.5 * calculateEta(tf_q_to_p).squaredNorm();

    //Store last squared norm as alpha
    alpha = 0.5 * calculateEta(tf_q_to_p).squaredNorm();
    std::cout << "delta after = " << delta << "\n";
    std::cout << "alpha = " << alpha << "\n";

    //Increment iteration
    i++;
  }

  std::cout << "Solution found after steps: " << i << " (max was " << nmax << ")\n";

  //Compute error and update best_error and corresponding solved_phi
  const auto current_error = calculateEta(tf_q_to_p);

  if (current_error.norm() < sol.error) {
    std::cout << "Updating best with error norm: " << current_error.norm() << "\n";
    sol.error_vector = current_error;
    sol.error = current_error.norm();
    std::vector<double> solved_phi_svec(phi.data(), phi.data() + phi.size());  // Temporary, TODO
    sol.solved_phi = solved_phi_svec;
  }

  //Pop last phi guess and recursively call this function with a new guess until all guesses are exhausted
  phi_starts.pop();
  if (!phi_starts.empty()) {
    return recursiveSearch(constraint, tf_m_to_q, phi_starts, sol);
  } else {
    return true;
  }
}

Eigen::Isometry3d productOfExponentials(
  const std::vector<ScrewAxis> & screws, const Eigen::VectorXd & phi, size_t start, size_t end)
{
  //recursively compute product of exponentials until the POE size is reduced to 1
  if (start < end)
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
  //Temporary, TODO
  // Eigen::VectorXd phi_low = Eigen::VectorXd::Map(lower_bounds.data(), lower_bounds.size());
  // Eigen::VectorXd phi_high = Eigen::VectorXd::Map(upper_bounds.data(), upper_bounds.size());
  // phi_bounds = {phi_low, phi_high};

  phi_starts = getGradStarts(lower_bounds_, upper_bounds_);
}

UnchainedScrews::UnchainedScrews(
  const std::vector<ScrewStamped> & screws, const std::vector<double> & lower_bounds,
  const std::vector<double> & upper_bounds, const Eigen::Isometry3d & tf_m_to_s)
: ScrewConstraint(screws, lower_bounds, upper_bounds, tf_m_to_s)
{
  //Temporary, TODO
  // Eigen::VectorXd phi_low = Eigen::VectorXd::Map(lower_bounds.data(), lower_bounds.size());
  // Eigen::VectorXd phi_high = Eigen::VectorXd::Map(upper_bounds.data(), upper_bounds.size());
  // phi_bounds = {phi_low, phi_high};

  phi_starts = getGradStarts(lower_bounds_, upper_bounds_);
}

bool UnchainedScrews::constraintFn(
  const Eigen::Isometry3d & tf_m_to_q, ScrewConstraintSolution & sol)
{
  if (size_ < 1) {
    return false;
  }

  sol.reset();

  auto starts = phi_starts;

  return recursiveSearch(this, tf_m_to_q, starts, sol);
}

bool UnchainedScrews::constraintFn(
  const Eigen::Isometry3d & tf_m_to_q, const std::vector<double> & phi_0,
  ScrewConstraintSolution & sol)
{
  if (size_ < 1) {
    return false;
  }

  sol.reset();
  auto starts = phi_starts;

  return recursiveSearch(this, tf_m_to_q, starts, sol);
}

// bool UnchainedScrews::constraintFn(ScrewConstraintInfo & sol)
// {
//   Eigen::VectorXd & phi = phi_starts.front();

//   //Check if sizes are valid, TODO: Might not need to implement this. Base class has it?
//   if ((screws.size() != phi.size()) || (screws.size() <= 0)) {
//     return false;
//   }

//   //Gradient descent parameters
//   const double gamma = 0.05;     //learning parameter
//   const size_t nmax = 100;       //max steps
//   const double epsilon = 0.001;  //converge limit

//   //Helper parameters
//   const Eigen::Isometry3d & tf_q_to_m = tf_m_to_q.inverse();

//   //Retrieve bounds
//   const Eigen::VectorXd & phi_min = phi_bounds.first;
//   const Eigen::VectorXd & phi_max = phi_bounds.second;

//   //Compute tf_q_to_p
//   const int m = screws.size();  //TODO
//   Eigen::Isometry3d pOE = productOfExponentials(screws, phi, 0, m - 1);
//   Eigen::Isometry3d tf_q_to_p = tf_q_to_m * pOE * tf_m_to_s;

//   //Compute alpha, the 1/2 squared norm we want to minimize
//   double alpha = 0.5 * calculateEta(tf_q_to_p).squaredNorm();

//   //Initialize iterating parameters
//   double delta = epsilon;
//   size_t i = 0;

//   //To clamp phi in the loop
//   auto eigen_clamp = [&phi](const Eigen::VectorXd & phi_min, const Eigen::VectorXd & phi_max) {
//     for (int i = 0; i < phi.size(); i++) phi[i] = std::clamp(phi[i], phi_min[i], phi_max[i]);
//     return phi;
//   };

//   while (i < nmax && fabs(delta) >= epsilon) {
//     //Compute phi
//     phi = phi - gamma * errorDerivative(tf_q_to_m, tf_m_to_s, phi, screws);

//     eigen_clamp(phi_min, phi_max);

//     //Compute tf_q_to_p
//     pOE = productOfExponentials(screws, phi, 0, m - 1);
//     tf_q_to_p = tf_q_to_m * pOE * tf_m_to_s;

//     //Compute new delta
//     delta = alpha - 0.5 * calculateEta(tf_q_to_p).squaredNorm();

//     //Store last squared norm as alpha
//     alpha = 0.5 * calculateEta(tf_q_to_p).squaredNorm();

//     //Increment iteration
//     i++;
//   }

//   //Compute error and update best_error and corresponding solved_phi
//   current_error = calculateEta(tf_q_to_p);

//   if (current_error.norm() < sol.error) {
//     sol.error_vector = current_error;
//     sol.error = current_error.norm();
//     std::vector<double> solved_phi_svec(phi.data(), phi.data() + phi.size());  // Temporary, TODO
//     sol.solved_phi = solved_phi_svec;
//   }

//   //Pop last phi guess and recursively call this function with a new guess until all guesses are exhausted
//   phi_starts.pop();
//   if (!phi_starts.empty()) {
//     return constraintFn(sol);
//   } else {
//     return true;
//   }
// }

// Eigen::Isometry3d UnchainedScrews::productOfExponentials(
//   std::vector<ScrewAxis> & screws, const Eigen::VectorXd & phi, int start, int end)
// {
//   //recursively compute product of exponentials until the POE size is reduced to 1
//   if (start < end)
//     return screws[start].getTF(phi[start]) * productOfExponentials(screws, phi, start + 1, end);

//   //when size is 1 return identity
//   else
//     return screws[0].getTF(0.0);
// }

std::queue<Eigen::VectorXd> UnchainedScrews::getGradStarts(
  const std::vector<double> & lower_bounds, const std::vector<double> & upper_bounds,
  double max_dist)
{
  std::queue<Eigen::VectorXd> output;

  Eigen::VectorXd guess1(2), guess2(2), guess3(2);
  guess1 << 0.5, 0.5;
  guess2 << 0.75, 0.25;
  guess3 << 1.0, 0.5;

  output.push(guess1);
  output.push(guess2);
  output.push(guess3);

  // const Eigen::VectorXd span = (phi_bounds.second - phi_bounds.first).cwiseAbs();
  // const size_t num_starts =
  //   ceil(span.maxCoeff() / max_dist) + 1;  //use the max span to determine number of starts
  // const Eigen::VectorXd real_step = span / num_starts;

  // Eigen::MatrixXd cond_grad_starts(
  //   size_,
  //   num_starts + 1);  //grad descent starts in condensed form with no regard to series constraint
  // Eigen::MatrixXd dist_grad_starts(
  //   size_,
  //   size_ * (num_starts) +
  //     1);  //grad descent starts in distributed form accounting for series constraint

  // for (size_t i = 0; i <= num_starts; ++i) {
  //   cond_grad_starts.col(i) = phi_bounds.first + i * real_step;
  // }

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

// // TODO, This function is unused at the moment
// Eigen::Isometry3d UnchainedScrews::getPose(const std::vector<double> & phi) const
// {
//   Eigen::Isometry3d output = Eigen::Isometry3d::Identity();

//   // Check input
//   if (phi.size() != size_) {
//     return output;
//   }

//   // Step through each axis and calculate
//   for (size_t i = 0; i < size_; ++i) {
//     output = output * axes_.at(i).getTF(phi[i]);
//   }
//   output = output * tf_m_to_s_;
//   return output;
// }

void UnchainedScrews::addScrewAxis(
  const ScrewStamped & axis, double lower_bound, double upper_bound)
{
  ScrewConstraint::addScrewAxis(axis, lower_bound, upper_bound);
  phi_starts = getGradStarts(lower_bounds_, upper_bounds_);
}

void UnchainedScrews::addScrewAxis(const ScrewAxis & axis, double lower_bound, double upper_bound)
{
  ScrewConstraint::addScrewAxis(axis, lower_bound, upper_bound);
  phi_starts = getGradStarts(lower_bounds_, upper_bounds_);
}

std::vector<ScrewStamped> UnchainedScrews::getVisualScrews() const
{
  std::vector<ScrewStamped> output;

  for (const auto & axis : axes_) {
    output.push_back(axis.toMsg());
  }

  return output;
}

}  // namespace affordance_primitives
