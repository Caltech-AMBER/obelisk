# Obelisk compliant message type to hold information generally obtained from joint encoders.

string MESSAGE_NAME="ObkJointEncoders"

std_msgs/Header header

builtin_interfaces/Time stamp   # TODO: remove

float64[] joint_pos
float64[] joint_vel
float64[] motor_surface_temps
float64[] motor_winding_temps
uint32[] motor_states           # per-motor fault/error code from the vendor low state (0 = OK); populated together with the temps
string[] joint_names
