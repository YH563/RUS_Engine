"""rus_sim_bridge 启动：前后端通信网关节点。"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("rus_sim_bridge")
    params_file = os.path.join(pkg_share, "config", "bridge_params.yaml")

    return LaunchDescription([
        Node(
            package="rus_sim_bridge",
            executable="rus_sim_bridge_node",
            name="bridge_node",
            output="screen",
            parameters=[params_file],
        ),
    ])
