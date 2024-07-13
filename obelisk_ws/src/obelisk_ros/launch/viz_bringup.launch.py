import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> None:
    """Generates the launch description."""
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    rviz_file_name = "rviz/basic_obk_config.rviz"
    rviz_config = urdf = os.path.join(
        get_package_share_directory('obelisk_ros'),
        rviz_file_name)
    
    urdf_file_name = 'urdf/g1.urdf'
    urdf = os.path.join(
        get_package_share_directory('g1_description'),
        urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()


    return LaunchDescription([
        # Node(
        #      package='tf2_ros',
        #      executable='static_transform_publisher',
        #      arguments = ['--x', '0', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id',
        #                    'world', '--child-frame-id', 'robot_world']
        # ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}]
        ),
        Node(
            package='obelisk_ros',
            executable='state_publisher',
            name='state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config]
        )
    ])
