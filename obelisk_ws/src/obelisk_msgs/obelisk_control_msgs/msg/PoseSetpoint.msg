string MESSAGE_NAME="PoseSetpoint"

std_msgs/Header header

# the desired pose should be in order [x, y, z, qw, qx, qy, qz]
float64[] u_mujoco
float64[] pose_des
