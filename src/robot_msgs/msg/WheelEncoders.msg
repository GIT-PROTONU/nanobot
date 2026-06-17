# Raw cumulative quadrature counts straight from the encoder node.
# Useful for debugging / dead-reckoning sanity checks; odometry is published
# separately as nav_msgs/Odometry by wheel_odometry.
std_msgs/Header header
int64 left_ticks
int64 right_ticks
# Measured wheel angular velocities (rad/s), derived from tick deltas.
float64 left_velocity
float64 right_velocity
