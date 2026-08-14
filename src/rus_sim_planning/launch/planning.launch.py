#!/usr/bin/env python3
"""
rus_sim_planning 规划节点启动文件
用法: ros2 launch rus_sim_planning planning.launch.py
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory("rus_sim_planning")
    config_path = os.path.join(pkg_share, "config", "planning_params.yaml")

    planning_node = Node(
        package="rus_sim_planning",
        executable="rus_sim_planning_node",
        name="planning_node",
        output="screen",
        parameters=[config_path],
    )

    return LaunchDescription([planning_node])
