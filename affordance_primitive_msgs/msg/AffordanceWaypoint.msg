# Defines a Cartesian waypoint for an affordance trajectory

# The duration after the trajectory start this waypoint should be reached in
duration time_from_start

# The pose of the waypoint. The header frame_id gives the frame this is defined with respect to
geometry_msgs/PoseStamped pose

# The velocity that is required at this waypoint. Note that this is given in the waypoint's frame (e.g. this twist is body-centric)
geometry_msgs/Twist twist

# The wrench that must be applied at this waypoint. Note that this is given in the waypoint's frame (e.g. this wrench is body-centric)
geometry_msgs/Wrench wrench
