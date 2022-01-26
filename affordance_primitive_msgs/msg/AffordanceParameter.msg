# Which dimensions are compliant and how much
CartesianBool admittance_dimensions
CartesianFloat admittance

# The wrench to apply
CartesianFloat apply_wrench

# Maximum allowable forces/torques on robot
CartesianFloat max_wrench
float64 max_force
float64 max_torque

# Maximum allowable robot velocities
CartesianFloat max_ee_velocity
float64[] max_joint_velocity

# EE Input and Output dimensions. More useful during teleoperation
CartesianBool ee_output_dimensions
CartesianBool ee_input_dimensions
