#!/usr/bin/env python3
"""rus_sim_recorder 记录节点启动文件

录什么（config/recorder_params.yaml）：
  通道 0  /driver/state       RobotState   机械臂全状态（125Hz）→ 复盘轨迹
  通道 1  /sensor/pointcloud  SensorFrame  压缩感知帧（当前帧 / 地图快照）→ 复盘点云

落盘位置：<output_dir>/<prefix>_<时间戳>.rusrec（默认 records/，绝不上传 / 不自动清理）

录制开关（运行期可控，见 docs/Protocol/WsProtocol.md §4.8）：
  /recorder/command：recorder_start（开始）/ recorder_stop（停录并封存）/ recorder_status（查询）
  默认 autostart=true 起来就录；autostart:=false 时起来待命，等 recorder_start。

用法:
  ros2 launch rus_sim_recorder recorder.launch.py                       # 默认录到 records/
  ros2 launch rus_sim_recorder recorder.launch.py output_dir:=/data/run01
  ros2 launch rus_sim_recorder recorder.launch.py autostart:=false       # 起来不录，等指令
  ros2 launch rus_sim_recorder recorder.launch.py enabled:=false         # 只跑节点不落盘（冒烟）
  ros2 service call /recorder/command rus_sim_interfaces/srv/CommandService "{command: recorder_start}"
  ros2 run rus_sim_recorder rus_sim_recorder_inspect records/run_*.rusrec

整系统一键（含录制，默认关）:
  ros2 launch rus_sim_bringup rus_sim.launch.py record:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("rus_sim_recorder")
    params_file = os.path.join(pkg_share, "config", "recorder_params.yaml")

    return LaunchDescription([
        DeclareLaunchArgument(
            "enabled", default_value="true",
            description="是否落盘（false = 只跑节点不写文件）"),
        DeclareLaunchArgument(
            "autostart", default_value="true",
            description="启动即录制（false = 起来待命，等 /recorder/command 的 recorder_start）"),
        DeclareLaunchArgument(
            "output_dir", default_value="records",
            description="输出目录（相对路径按启动工作目录解析）"),
        Node(
            package="rus_sim_recorder",
            executable="rus_sim_recorder_node",
            name="recorder_node",
            output="screen",
            parameters=[params_file, {
                "enabled": LaunchConfiguration("enabled"),
                "autostart": LaunchConfiguration("autostart"),
                "output_dir": LaunchConfiguration("output_dir"),
            }],
        ),
    ])
