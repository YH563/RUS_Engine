#!/usr/bin/env python3
"""rus_sim_perception 感知节点启动文件

数据源（source 参数，详见 config/perception_params.yaml）：
  realsense  RealSense 直连（librealsense2；设备独占，勿与 realsense2_camera 并存）
  ros_topic  PointCloud2 话题（realsense2_camera / 仿真 / rosbag）
  replay     离线 PCD 循环回放（无设备联调）
  auto       探测：有 RealSense 设备 → realsense，否则 → ros_topic

用法:
  ros2 launch rus_sim_perception perception.launch.py                    # 默认 auto（本文件传入空值）
  ros2 launch rus_sim_perception perception.launch.py source:=realsense  # 强制直连
  ros2 launch rus_sim_perception perception.launch.py source:=ros_topic  # 走 realsense2_camera 话题
  ros2 launch rus_sim_perception perception.launch.py source:=replay replay_path:=/tmp/test_cloud.pcd
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("rus_sim_perception")
    params_file = os.path.join(pkg_share, "config", "perception_params.yaml")

    return LaunchDescription([
        DeclareLaunchArgument(
            "source", default_value="",
            description="数据源覆盖：空=auto（有设备走直连，否则回落 ros_topic）；"
                        "realsense / ros_topic / replay"),
        DeclareLaunchArgument(
            "replay_path", default_value="",
            description="source=replay 时的 PCD 文件或目录"),
        Node(
            package="rus_sim_perception",
            executable="rus_sim_perception_node",
            name="perception_node",
            output="screen",
            parameters=[params_file, {
                "source": LaunchConfiguration("source"),
                "replay_path": LaunchConfiguration("replay_path"),
            }],
        ),
    ])
