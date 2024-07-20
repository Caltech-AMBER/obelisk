# We expect the shape of the image tensor to be (..., H, W, C), where "..." denotes batch dimensions of the user's
# choice, and C is ordered RGB. If there are alpha channels, they are a fourth channel, and if there are depth channels,
# they follow all colors and alpha. Note that this is only a convention that we declare - this is not enforced!
string MESSAGE_NAME="ObkImage"
obelisk_std_msgs/UInt8MultiArray y
