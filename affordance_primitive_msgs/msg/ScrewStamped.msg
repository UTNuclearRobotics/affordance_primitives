# This defines a screw axis with respect to the frame given in the header
# A screw axis is essentially a unit vector (passing through some origin point) that defines the center of rotation

# The frame in the header is the "moving frame"
# We will be calculating its velocity from the given screw
std_msgs/Header header

# This is the origin of the screw, w.r.t. the moving frame
geometry_msgs/Point origin

# This is the axis of rotation, w.r.t. the moving frame
geometry_msgs/Vector3 axis

# The screw pitch relates the rotation to linear motion along the axis
# If pitch = Infinity, there is no rotation and this is a pure translation case
# Setting is_pure_translation = True ignores pitch
bool is_pure_translation
float64 pitch
