# We expect the shape of the image tensor to be (..., H, W, C), where "..." denotes batch dimensions of the user's
# choice, and C is 3 for RGB, 1 for depth maps, and 1 for segmentation. Note that this is only a convention that we
# declare - this is not enforced!
string MESSAGE_NAME="ObkImage"
obelisk_std_msgs/UInt8MultiArray rgb
obelisk_std_msgs/Float32MultiArray depth
obelisk_std_msgs/UInt8MultiArray segmentation
