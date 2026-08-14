#!/usr/bin/env python3
"""rus_sim_perception 感知层启动：点云 / 视觉 / 融合节点（预留占位）。

当前节点实现为空（占位），此 launch 保留结构，感知模块落地后可直接使用。

用法：
  ros2 launch rus_sim_perception perception.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("rus_sim_perception")
    params_file = os.path.join(pkg_share, "config", "perception_params.yaml")

    return LaunchDescription([
        Node(
            package="rus_sim_perception",
            executable="rus_sim_perception_node",
            name="perception_node",
            output="screen",
            parameters=[params_file],
        ),
    ])
