#include <affordance_primitives/screw_planning/screw_planning.hpp> 
namespace affordance_primitives
{
Eigen::VectorXd errorDerivative(
  const Eigen::Isometry3d & tf_q_to_m, const Eigen::Isometry3d & tf_m_to_s,
  const Eigen::VectorXd phi_current, const std::vector<ScrewAxis>& screw_axis_set)
{
//Algorithm
//Require: Set of screw axes in frame M, screw_axis_set
//Require: Set of screw angles, phi
//Require: Transforms, tf_q_to_m and tf_m_to_s
//Compute lambda

//Compute tf_q_to_p
  const Eigen::Isometry3d pOE = productOfExponentials (screw_axis_set, phi_current, screw_axis_set.size(), 0, screw_axis_set.size()-1);
  const Eigen::Isometry3d tf_q_to_p = tf_q_to_m * pOE * tf_m_to_s;
//Get eta of tf_q_to_p
  const Eigen::VectorXd xi = eta(tf_q_to_p);
//Construct E and psi matrices
  int m = screw_axis_set.size();
  Eigen::MatrixXd E(m,6*m);
  Eigen::VectorXd Psi(6*m,1);

for (int j=0; j<=screw_axis_set.size(); j++)
{
E.row(j)<< xi.transpose(), Eigen::MatrixXd::Zero(1,m-6*j);
Eigen::Isometry3d pOE_left = productOfExponentials (screw_axis_set, phi_current, screw_axis_set.size(), 0, j );//TODO: Move declaration out of the loop
Eigen::Isometry3d pOE_right = productOfExponentials (screw_axis_set, phi_current, screw_axis_set.size(), j+1, m );
Eigen::VectorXd jScrewAxis(6,1);
jScrewAxis <<screw_axis_set[j].getAxis(), screw_axis_set[j].getLinearVector();
Eigen::Isometry3d meu = tf_q_to_m*pOE_left*affordance_primitives::getSkewSymmetricMatrix(jScrewAxis)*pOE_right*tf_m_to_s; 
Psi.segment(6*j,6) = eta(meu);
}

Eigen::VectorXd Lambda(m,1);
Lambda = E*Psi;

  return Lambda;
}

<<<<<<< HEAD
std::pair<double, Eigen::Isometry3d> runGradientDescent(
  const Eigen::Isometry3d & tf_m_to_q, const Eigen::Isometry3d & tf_m_to_e,
  const double theta_start, const std::pair<double, double> theta_limits,
  const ScrewAxis & screw_axis)
=======

bool constraintFn(
  const Eigen::Isometry3d & tf_m_to_q, const Eigen::Isometry3d & tf_m_to_s, const std::vector<ScrewAxis>& screw_axis_set, const Eigen::VectorXd phi_max, const Eigen::VectorXd phi_start,
  Eigen::Ref<Eigen::VectorXd> final_error)
>>>>>>> 6c6b11c... clean up and redef constraintFn to match paper algorithm
{
//Gradient descent parameters
  // TODO: tune these
  /* const Eigen::VectorXd gamma = Eigen::MatrixXd::Constant(phi_start.size(), 1, 0.05); */
  const double gamma = 0.05;
  const size_t nmax = 100;//max steps
  const double epsilon = 0.001;//converge limit
//Compute tf_q_to_m
  const Eigen::Isometry3d tf_q_to_m = tf_m_to_q.inverse();
//Sample random phi
  Eigen::VectorXd phi = phi_start; //phi_start is already randomly sampled and fed to this function
//Compute tf_q_to_p
  Eigen::Isometry3d pOE = productOfExponentials (screw_axis_set, phi_start, screw_axis_set.size(), 0,screw_axis_set.size()-1);
  Eigen::Isometry3d tf_q_to_p = tf_m_to_q.inverse() * pOE * tf_m_to_s;
//Compute alpha, the 1/2 squared norm we want to minimize
  double alpha = 0.5 * eta(tf_q_to_p).squaredNorm();

//Initialize iterating parameters
  size_t i = 0;
  double delta = epsilon;

<<<<<<< HEAD
  Eigen::Isometry3d tf_q_to_path = tf_m_to_q.inverse() * screw_axis.getTF(theta) * tf_m_to_e;
  Eigen::VectorXd error = calcError(tf_q_to_path);
  double error_norm_diff = std::numeric_limits<double>::max();

  // TODO: use squaredNorm() instead of norm() for faster performance
  while (fabs(error_norm_diff) > converge_limit && i < max_steps) {
    i++;
    const double last_error_norm = error.norm();
    theta -= gamma * calcErrorDerivative(tf_m_to_q, tf_m_to_e, theta, screw_axis);
    error = calcError(tf_m_to_q.inverse() * screw_axis.getTF(theta) * tf_m_to_e);
    error_norm_diff = last_error_norm - error.norm();

    if (theta < theta_limits.first) {
      theta = theta_limits.first;
      break;
    }
    if (theta > theta_limits.second) {
      theta = theta_limits.second;
      break;
    }
=======
  while (i < nmax && fabs(delta) >= epsilon ) {
//Compute phi
	 phi = phi - gamma * errorDerivative(tf_q_to_m, tf_m_to_s, phi, screw_axis_set); 
//Clamp phi
    phi = clamp(phi,phi.size(), phi_max); 
//Compute tf_q_to_p
  Eigen::Isometry3d pOE = productOfExponentials (screw_axis_set, phi_start, screw_axis_set.size(), 0,screw_axis_set.size()-1);
  Eigen::Isometry3d tf_q_to_p = tf_m_to_q.inverse() * pOE * tf_m_to_s;
//Compute new delta
	 delta = alpha - 0.5 * eta(tf_q_to_p).squaredNorm();
//store last squared norm as alpha
  alpha = 0.5 * eta(tf_q_to_p).squaredNorm();
  i++;
>>>>>>> 6c6b11c... clean up and redef constraintFn to match paper algorithm
  }

  final_error = eta(tf_q_to_p);

  return true;
}

<<<<<<< HEAD
std::queue<double> getGradStarts(const std::pair<double, double> & limits, double max_dist)
{
  std::queue<double> output;

  const double span = fabs(limits.second - limits.first);
  const size_t num_starts = ceil(span / max_dist) + 1;
  const double real_step = span / num_starts;

  for (size_t i = 0; i < num_starts; ++i) {
    output.push(limits.first + i * real_step);
  }
  output.push(limits.second);

  return output;
}

bool screwConstraint(
  const Eigen::Isometry3d & current_pose, const Eigen::Isometry3d & start_pose,
  const ScrewAxis & screw_axis, const std::pair<double, double> theta_limits, double theta_guess,
  Eigen::Ref<Eigen::VectorXd> out)
{
  // Find the closest point on the path
  std::pair<double, Eigen::Isometry3d> best_output =
    runGradientDescent(current_pose, start_pose, theta_guess, theta_limits, screw_axis);
  double best_error = calcError(best_output.second).norm();
  auto start_guesses = getGradStarts(theta_limits);
  while (!start_guesses.empty() && best_error > 5e-3) {
    const auto closest_pt =
      runGradientDescent(current_pose, start_pose, start_guesses.front(), theta_limits, screw_axis);
    start_guesses.pop();

    const double current_error = calcError(closest_pt.second).norm();
    if (current_error < best_error) {
      best_output = closest_pt;
      best_error = current_error;
    }
  }

  // Use closest point to calculate error
  auto error = calcError(best_output.second);
  out = error;
=======

Eigen::Isometry3d productOfExponentials (const std::vector<ScrewAxis>& screwAxisSet, const std::vector<double> phi, int size, int start, int end)
{
assert(&screwAxisSet != NULL && size > 0);
if ((size > 1) && (start<=end))
    return screwAxisSet[start].getTF(phi[start])*productOfExponentials(screwAxisSet, phi, size-1, start+1, end);
else
    return screwAxisSet[0].getTF(0.0);//identity
}

>>>>>>> 6c6b11c... clean up and redef constraintFn to match paper algorithm

Eigen::VectorXd eta (const Eigen::Isometry3d & tf){
  const Eigen::AngleAxisd axisAngleRep(tf.rotation());
  const Eigen::Vector3d lin = tf.linear();
  Eigen::VectorXd eta(6);
  eta.head(3) = lin;
  eta.tail(3) = axisAngleRep.angle()* axisAngleRep.axis();
  return eta;

}

// Function to clamp the elements in given range
Eigen::VectorXd clamp(const Eigen::VectorXd arr, const int size, const Eigen::VectorXd arr_high)
{
	Eigen::VectorXd arr_clamped(size);
	//function assumes lows are zeros. TODO: implement arbitrary lows
	Eigen::VectorXd arr_low = Eigen::VectorXd::Zero(size,1);
	for(int i = 0; i < size; i++)
		arr_clamped[i] = std::clamp(arr[i], arr_low[i], arr_high[i]);
	return arr_clamped;
}
    }  // namespace affordance_primitives
