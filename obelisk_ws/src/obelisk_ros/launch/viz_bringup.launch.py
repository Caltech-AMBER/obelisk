import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> None:
    """Generates the launch description."""
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    rviz_file_name = "rviz/basic_obk_config.rviz"
    rviz_config = urdf = os.path.join(get_package_share_directory("obelisk_ros"), rviz_file_name)

    urdf_file_name = "urdf/g1.urdf"
    urdf = os.path.join(get_package_share_directory("g1_description"), urdf_file_name)
    with open(urdf, "r") as infp:
        robot_desc = infp.read()

    return LaunchDescription(
        [
            Node(
                package="obelisk_viz_cpp",
                executable="default_robot_viz",
                name="robot_viz",
                output="screen",
                parameters=[
                    {
                        "robot_viz_urdf_path_param": urdf,
                        "robot_viz_pub_viz_joint_settings": "topic:joint_states",
                        "robot_viz_sub_viz_est_settings": "topic:estimated_state",
                        "robot_viz_timer_viz_joint_settings": "timer_period_sec:0.05",
                    }
                ],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time, "robot_description": robot_desc}],
            ),
            Node(
                package="obelisk_ros",
                executable="state_publisher",
                name="state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_desc}],
            ),
            Node(package="rviz2", executable="rviz2", name="rviz2", output="screen", arguments=["-d", rviz_config]),
        ]
    )
