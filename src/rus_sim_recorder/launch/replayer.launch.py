#!/usr/bin/env python3
"""rus_sim_recorder 回放节点启动文件

把 .rusrec 按时间轴重发回录制时的话题（默认原样，可用 topic_prefix 隔离）：

  通道 0  /driver/state       RobotState   → 前端「回放」可视化（bridge 已在订阅）
  通道 1  /sensor/pointcloud  SensorFrame  → 前端点云回放（bridge /sensor 通道）

指令（前端 → bridge → /replayer/command，见 docs/Protocol/WsProtocol.md §4.7）：
  replay_list / replay_load / replay_start / replay_pause / replay_resume /
  replay_stop / replay_seek / replay_set_speed / replay_step / replay_status

用法:
  ros2 launch rus_sim_recorder replayer.launch.py                            # 载入 records/ 里第 0 个
  ros2 launch rus_sim_recorder replayer.launch.py file_path:=records/run_20260926_101500.rusrec
  ros2 launch rus_sim_recorder replayer.launch.py autoplay:=true speed:=2.0  # 启动即 2 倍速播放
  ros2 launch rus_sim_recorder replayer.launch.py topic_prefix:=/replay      # 与真机共存（隔离话题）

⚠️ topic_prefix 为空时回放直接发 /driver/state 等录制时话题：真驱动在线会撞话题，
   请先停驱动（只留 bridge）或用 topic_prefix 隔离。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("rus_sim_recorder")
    params_file = os.path.join(pkg_share, "config", "replayer_params.yaml")

    return LaunchDescription([
        DeclareLaunchArgument(
            "record_dir", default_value="records",
            description="录音目录（相对路径按启动工作目录解析）"),
        DeclareLaunchArgument(
            "file_path", default_value="",
            description="直接指定录音文件（非空时忽略 file_index）"),
        DeclareLaunchArgument(
            "file_index", default_value="0",
            description="目录内按文件名升序的第 N 个文件"),
        DeclareLaunchArgument(
            "autoplay", default_value="false",
            description="启动即播放（false = 载入后停在起点，等 replay_start）"),
        DeclareLaunchArgument(
            "speed", default_value="1.0",
            description="初始倍速（0.05 ~ 20）"),
        DeclareLaunchArgument(
            "topic_prefix", default_value="",
            description="发布话题前缀（\"\" = 原样；隔离用 /replay）"),
        Node(
            package="rus_sim_recorder",
            executable="rus_sim_recorder_replay",
            name="replayer_node",
            output="screen",
            parameters=[params_file, {
                "record_dir": LaunchConfiguration("record_dir"),
                "file_path": LaunchConfiguration("file_path"),
                "file_index": LaunchConfiguration("file_index"),
                "autoplay": LaunchConfiguration("autoplay"),
                "speed": LaunchConfiguration("speed"),
                "topic_prefix": LaunchConfiguration("topic_prefix"),
            }],
        ),
    ])
