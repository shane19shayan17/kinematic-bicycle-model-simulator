# Timestamp and frame metadata. The simulator uses the "map" frame.
std_msgs/Header header

# Position in meters.
float64 x
float64 y

# Heading in radians, counter-clockwise from +X.
float64 heading

# Applied actuator values.
float64 linear_velocity
float64 steering_angle
