# Defines a Cartesian trajectory

# Every waypoint, wrench, twist, etc is defined in this frame
std_msgs/Header header

# The trajectory
AffordanceWaypoint[] trajectory
