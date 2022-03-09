# The addmittance is added in each Cartesian direction as:
# velocity = addmittance * force
CartesianFloat admittance

# Maximum allowable forces/torques on robot
CartesianFloat max_wrench
float64 max_force
float64 max_torque

# Maximum allowable robot velocities
CartesianFloat max_ee_velocity
float64[] max_joint_velocity
