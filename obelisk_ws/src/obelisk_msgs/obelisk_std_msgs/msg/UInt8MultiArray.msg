# This message type is meant to be a "fixed" version of the deprecated Float64MultiArray message type
# Original message type: http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Float64MultiArray.html
# Github issue discussing striding: https://github.com/ros/std_msgs/issues/8
# We follow the striding convention discussed in the above issue:
# multiarray(i,j,k) = data[data_offset + dim_stride[0]*i + dim_stride[1]*j + dim_stride[2]*k]

string MESSAGE_NAME="UInt8MultiArray"

std_msgs/MultiArrayLayout layout
uint8[] data
