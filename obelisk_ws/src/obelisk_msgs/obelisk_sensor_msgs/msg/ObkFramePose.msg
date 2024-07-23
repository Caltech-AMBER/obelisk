# Obelisk complient message type to hold the current pose of a frame/body.

string MESSAGE_NAME="ObkFramePose"

std_msgs/Header header

string frame_name
geometry_msgs/Quaternion orientation
geometry_msgs/Vector3 position
