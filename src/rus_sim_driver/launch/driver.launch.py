#!/usr/bin/env python3
"""rus_sim_driver 驱动层启动：驱动节点 + robot_state_publisher。

RViz 已注释（联调阶段不需要 GUI，见文件末尾；恢复时取消两处注释即可）。

用法：
  ros2 launch rus_sim_driver driver.launch.py
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('rus_sim_driver')

    urdf_path = os.path.join(pkg_dir, 'robot_model', 'fairino3_v6.urdf')
    # RViz 配置（随 RViz 一起注释；恢复 RViz 时取消本行注释）
    # rviz_config = os.path.join(pkg_dir, 'config', 'simulation.rviz')
    params_file = os.path.join(pkg_dir, 'config', 'driver_params.yaml')

    return LaunchDescription([

        # ── 驱动节点 ──
        Node(
            package='rus_sim_driver',
            executable='rus_sim_driver_node',
            name='driver_node',
            parameters=[params_file],
        ),

        # ── robot_state_publisher ──
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': open(urdf_path, 'r').read(),
            }],
        ),

        # ── RViz（已注释：不启动 GUI；恢复时取消下方注释，并恢复上方 rviz_config）──
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', rviz_config],
        # ),
    ])
