# Low-level, post-kinematics command the motor_control node actually applies.
# Range [-1.0, 1.0] per side; sign = direction. Published for introspection so
# you can see exactly what hit the PCA9685.
std_msgs/Header header
float32 left   # -1.0 .. 1.0
float32 right  # -1.0 .. 1.0
