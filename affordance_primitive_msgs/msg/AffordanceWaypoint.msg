# Defines a Cartesian waypoint for an affordance trajectory

# The duration after the trajectory start this waypoint should be reached in
duration time_from_start

# The pose of the waypoint
geometry_msgs/Pose pose

# The velocity that is required at this waypoint
geometry_msgs/Twist twist

# The wrench that must be applied at this waypoint
geometry_msgs/Wrench wrench
