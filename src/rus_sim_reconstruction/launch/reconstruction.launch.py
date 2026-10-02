import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory("rus_sim_reconstruction"),
        "config", "reconstruction_params.yaml")
    return LaunchDescription([
        Node(
            package="rus_sim_reconstruction",
            executable="rus_sim_reconstruction_node",
            name="reconstruction_node",
            output="screen",
            parameters=[params],
        ),
    ])
