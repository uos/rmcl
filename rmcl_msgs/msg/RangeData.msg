# Computed ranges of a model's ray buffer
float32[]                ranges # required
bool[]                   mask
# We can attach a list of attributes to the ray buffer
# An zero length list marks a missing attribute
geometry_msgs/Point32[]  normals
std_msgs/ColorRGBA[]     colors
float32[]                stamps
float32[]                intensities
uint32[]                 labels # eg, obj id