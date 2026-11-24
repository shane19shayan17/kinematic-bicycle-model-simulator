import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the simulator with its installed YAML configuration."""
    config_path = os.path.join(
        get_package_share_directory("sim"), "config", "sim.yaml"
    )

    return LaunchDescription([
        Node(
            package="sim",
            executable="sim",
            name="sim",
            output="screen",
            parameters=[config_path],
        )
    ])
