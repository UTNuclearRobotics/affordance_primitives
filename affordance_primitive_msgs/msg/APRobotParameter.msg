# The addmittance is added in each Cartesian direction as:
# velocity = addmittance * force
CartesianFloat admittance

# Maximum allowable forces/torques on robot
# If not set, there is no limit
CartesianFloat max_wrench
float64 max_force
float64 max_torque

# Maximum allowable robot velocities
# If not set, there is no limit
CartesianFloat max_ee_velocity
float64[] max_joint_velocity
