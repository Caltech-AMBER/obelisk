# Obelisk compliant message type to hold IMU information.
# This message type generally mimics the ROS2 sensor_msgs/msg/imu type.

string MESSAGE_NAME="ObkImu"

std_msgs/Header header

geometry_msgs/Quaternion orientation

# This is the covariance matrix for elements of the tangent space.
# This means that this represents a distribution about the identity element of a quaternion in the tangent space.
# To convert from the tangent space to a quaternion element you must use an exp map.
float64[9] orientation_covariance


geometry_msgs/Vector3 angular_velocity
float64[9] angular_velocity_covariance # Row major about x, y, z axes


geometry_msgs/Vector3 linear_acceleration
float64[9] linear_acceleration_covariance # Row major x, y z
