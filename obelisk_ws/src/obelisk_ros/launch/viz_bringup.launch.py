import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

# ------------------------------------------------------------------- #
# ---------------- This file is for testing purposes ---------------- #
# ------------------------------------------------------------------- #


def generate_launch_description() -> None:
    """Generates the launch description."""
    urdf_file_name = "urdf/g1.urdf"
    urdf = os.path.join(get_package_share_directory("g1_description"), urdf_file_name)
    with open(urdf, "r") as infp:
        robot_desc = infp.read()

    urdf_file_name = "urdf/go2_description.urdf"
    urdf = os.path.join(get_package_share_directory("go2_description"), urdf_file_name)
    with open(urdf, "r") as infp:
        robot_desc2 = infp.read()

    return LaunchDescription(
        [
            Node(
                package="obelisk_ros",
                executable="state_publisher",
                name="state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_desc, "robot_description2": robot_desc2}],
            ),
        ]
    )
