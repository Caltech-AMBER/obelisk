# Obelisk compliant message type to hold information generally obtained from joint encoders.

string MESSAGE_NAME="ObkJointEncoders"

std_msgs/Header header

builtin_interfaces/Time stamp   # TODO: remove

float64[] joint_pos
float64[] joint_vel
float64[] motor_surface_temps
float64[] motor_winding_temps
string[] joint_names
