#include <affordance_primitives/screw_planning/screw_planning.hpp> 
namespace affordance_primitives
{
Eigen::VectorXd calcError(const Eigen::Isometry3d & tf_err)
{
  Eigen::VectorXd error(6);
  error.setZero();

  const Eigen::AngleAxisd angle_err(tf_err.linear());
  error.head(3) = tf_err.translation();
  error.tail(3) = angle_err.angle() * angle_err.axis();

  return error;
}

Eigen::VectorXd calcErrorDerivative(
  const Eigen::Isometry3d & tf_m_to_q, const Eigen::Isometry3d & tf_m_to_s,
  const std::vector<double> phi_current, const std::vector<Eigen::VectorXd> screw_axis_set)
{
//Algorithm
//Require: Set of screw axes in frame M, screw_axis_set
//Require: Set of screw angles, phi
//Require: Transforms, tf_q_to_m and tf_m_to_s
//Compute lambda
const Eigen::Isometry3d tf_q_to_m = tf_m_to_q.inverse();

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
Eigen::Isometry3d meu = tf_q_to_m*pOE_left*affordance_primitives::getSkewSymmetricMatrix(screw_axis_set[j])*pOE_right*tf_m_to_s; 
Psi.segment(6*j,6) = eta(meu);
}

Eigen::VectorXd Lambda(m,1);
Lambda = E*Psi;

  return Lambda;
}

std::pair<double, Eigen::Isometry3d> findClosestPoint(
  const Eigen::Isometry3d & tf_m_to_q, const Eigen::Isometry3d & tf_m_to_s, const std::vector<double> phi_start, const std::vector<double> phi_max, const std::vector<ScrewAxis> & screw_axis_set)
{
  // TODO: tune these
  const std::vector<double> gamma(phi_start.size(), 0.05);
  const size_t max_steps = 100;
  const double converge_limit = 0.001;

  std::vector<double> phi = phi_start;
  size_t i = 0;

  const Eigen::Isometry3d pOE = productOfExponentials (screw_axis_set, phi_start, screw_axis_set.size(), 0);
  Eigen::Isometry3d tf_q_to_p = tf_m_to_q.inverse() * pOE * tf_m_to_s;
  Eigen::VectorXd error = calcError(tf_q_to_p);
  double error_norm_diff = 2 * converge_limit;

  // TODO: use squaredNorm() instead of norm() for faster performance
  while (fabs(error_norm_diff) > converge_limit && i < max_steps) {
    i++;
    const double last_error_norm = error.norm();
    phi -= gamma * calcErrorDerivative(tf_m_to_q, tf_m_to_s, phi, screw_axis_set);
    error = calcError(tf_m_to_q.inverse() * screw_axis.getTF(theta) * tf_m_to_e);
    error_norm_diff = last_error_norm - error.norm();

    if (theta < 0) {
      theta = 0;
      break;
    }
    if (theta > theta_max) {
      theta = theta_max;
      break;
    }
  }
  tf_q_to_path = tf_m_to_q.inverse() * screw_axis.getTF(theta) * tf_m_to_e;
  return std::make_pair(theta, tf_q_to_path);
}

bool constraintFn(
  const Eigen::Isometry3d & tf_m_to_q, const Eigen::Isometry3d & tf_m_to_s, const std::vector<Eigen::VectorXd> & screwAxisSet, std::vector<double> phi_max, std::vector<double> phi_guess,
  Eigen::Ref<Eigen::VectorXd> phi_out)//Data type of phi_out?
{
  // Find the closest point on the path
  const auto closest_pt =
    findClosestPoint(tf_m_to_q, tf_m_to_s, phi_guess, phi_max, screwAxisSet);

  // Use closest point to calculate error
  auto error = calcError(closest_pt.second);
  phi_out = error;

  return true;
}


Eigen::Isometry3d productOfExponentials (const std::vector<Eigen::VectorXd>& screwAxisSet, const std::vector<double> phi, int size, int start, int end)
{
assert(screwAxisSet != NULL && size > 0);
if ((size > 1) && (start<=end))
    return screwAxisSet[start].getTF(phi[start])*productOfExponentials(screwAxisSet, phi, size-1, start+1, end);
else
    return screwAxisSet[0].getTF(0.0);//identity
}


Eigen::VectorXd eta (const Eigen::Isometry3d & tf){
  const Eigen::AngleAxisd axisAngleRep(tf.rotation());
  const Eigen::Vector3d lin = tf.linear();
  Eigen::VectorXd eta(6);
  eta.head(3) = lin;
  eta.tail(3) = axisAngleRep.angle()* axisAngleRep.axis();
  return eta;

}
    }  // namespace affordance_primitives
