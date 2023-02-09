#include <affordance_primitives/screw_planning/screw_planning.hpp>
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
    Xi.row(j) << Eigen::MatrixXd::Zero(1, j * 6), xi.transpose(),
      Eigen::MatrixXd::Zero(1, (m - 1 - j) * 6);

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
  /* std::cout << "error is: " << screw_constraint_info.error.norm() << std::endl; */
  phi_starts.pop();
  /* std::cout << "Error is: " << screw_constraint_info.error.norm() << std::endl; */

  if (screw_constraint_info.error.norm() < screw_constraint_info.best_error) {
    std::cout << "Inside error changer: Error is: " << screw_constraint_info.error.norm()
              << " and Best Error is: " << screw_constraint_info.best_error << std::endl;
    screw_constraint_info.best_error = screw_constraint_info.error.norm();
  }

  //Recursively call this function with a new guess for phi until the following conditions are met
  /* if (!phi_starts.empty() || (screw_constraint_info.best_error > 5e-3)) { */
  if (!phi_starts.empty()) {
    phi = phi_starts.front();
    /* std::cout << "Recursive calls" << std::endl; */
    return constraintFn(screw_constraint_info);
  } else {
    std::cout << "Hit end of recursion with error: " << screw_constraint_info.error.norm()
              << " and Best Error: " << screw_constraint_info.best_error << std::endl;
    return true;
  }
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
  /* Eigen::VectorXd phi_low(3); */
  /* phi_low << 0.25, 0.5, 1; */
  /* Eigen::VectorXd phi_high(3); */
  /* phi_high << 1.25, 1.5, 2; */

  /* const std::pair<Eigen::VectorXd, Eigen::VectorXd> phi_boundst = {phi_low, phi_high}; */
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

  /* size_t row_init_size; */
  bool chained = true;

  if (chained) {
    //reshape to distributed form
    for (int i = 0; i < screw_set_size; i++) {
      /* Eigen::RowVectorXd holder; */
      //fill mins and active screw sample
      /* dist_grad_starts.row(i) << Eigen::MatrixXd::Constant( */
      /*   1, (num_starts * i), cond_grad_starts.coeff(i, 0)), */
      /*   cond_grad_starts.row(i); */
      /* holder.conservativeResize(num_starts * i + cond_grad_starts.row(0).size()); */
      /* holder << Eigen::RowVectorXd::Constant((num_starts * i), cond_grad_starts.coeff(i, 0)), */
      /*   cond_grad_starts.row(i); */
      /* row_init_size = num_starts * i + cond_grad_starts.row(0).size(); */
      /* dist_grad_starts.block(i, 0, 1, row_init_size) */
      /*   << Eigen::MatrixXd::Constant(1, (num_starts * i), cond_grad_starts.coeff(i, 0)), */
      /*   cond_grad_starts.row(i); */
      /* for (int j = 0; j <= i; j++) { */
      //fill max elements
      /* dist_grad_starts.row(i) << Eigen::MatrixXd::Constant( */
      /*   1, num_starts * (screw_set_size - 1 - i), */
      /*   cond_grad_starts.coeff(j, cond_grad_starts.row(0).size() - 1)); */
      /* holder.conservativeResize(holder.size() + num_starts * (screw_set_size - 1 - i)); */
      /* row_init_size = num_starts * (screw_set_size - 1 - i); */
      /* holder << Eigen::RowVectorXd::Constant( */
      /*   num_starts * (screw_set_size - 1 - i), */
      /*   cond_grad_starts.coeff(j, cond_grad_starts.row(0).size() - 1)); */
      /* dist_grad_starts.block(i, row_init_size - 1, 1, num_starts * (screw_set_size - 1 - i) + 1) */
      /*   << Eigen::RowVectorXd::Constant( */
      /*        num_starts * (screw_set_size - 1 - i) + 1, */
      /*        cond_grad_starts.coeff(i, cond_grad_starts.row(0).size() - 1)); */
      /* } */
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
}  // namespace affordance_primitives
