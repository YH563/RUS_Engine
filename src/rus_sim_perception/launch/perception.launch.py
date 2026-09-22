#!/usr/bin/env python3
"""rus_sim_perception 感知节点启动文件

数据源（source 参数，详见 config/perception_params.yaml）：
  realsense  RealSense 直连（librealsense2；设备独占，勿与 realsense2_camera 并存）
  ros_topic  PointCloud2 话题（realsense2_camera / 仿真 / rosbag）
  replay     离线 PCD 循环回放（无设备联调）
  auto       探测：有 RealSense 设备 → realsense，否则 → ros_topic

联调测试点云（input_pcd 参数）：
  启动即加载的 PCD，按 **base_link 系**处理（不做坐标变换，也不需要 /driver/state 位姿），
  默认指向 ~/.rus_sim/test_data/test_cloud.pcd（rus_sim_gen_test_cloud 生成的起伏扫查面）。
  ⚠️ 加载后会作为地图种子进入建图并对外发布 /sensor/pointcloud；不需要时用开关关闭。
  ⚠️ 文件不存在时默认值自动置空（不会启动即报错）。

用法:
  ros2 launch rus_sim_perception perception.launch.py                        # 默认 auto + 加载测试点云
  ros2 launch rus_sim_perception perception.launch.py load_test_cloud:=false  # 关闭测试点云（只看真实源）
  ros2 launch rus_sim_perception perception.launch.py input_pcd:=/abs/path/to/other.pcd  # 换一份点云
  ros2 launch rus_sim_perception perception.launch.py source:=realsense       # 强制直连
  ros2 launch rus_sim_perception perception.launch.py source:=ros_topic  # 走 realsense2_camera 话题
  ros2 launch rus_sim_perception perception.launch.py source:=replay replay_path:=/tmp/test_cloud.pcd
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

# 联调测试点云的持久位置（不受 /tmp 清理影响）；文件缺失时自动关闭该参数
TEST_CLOUD_PATH = os.path.expanduser("~/.rus_sim/test_data/test_cloud.pcd")
DEFAULT_INPUT_PCD = TEST_CLOUD_PATH if os.path.isfile(TEST_CLOUD_PATH) else ""


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
        # 联调测试点云：默认加载 ~/.rus_sim/test_data/test_cloud.pcd（base_link 系，不做位姿对齐）。
        # 关闭方式用 load_test_cloud:=false —— 注意 `input_pcd:=""` 在 ros2 launch 下是
        # malformed argument（引号被 shell 剥掉后变成 `input_pcd:=`），不可用。
        DeclareLaunchArgument(
            "load_test_cloud", default_value="true",
            description="启动时是否加载联调测试点云（路径见 input_pcd）；"
                        "false = 不加载，此时只有 source 指定的数据源供数"),
        DeclareLaunchArgument(
            "input_pcd", default_value=DEFAULT_INPUT_PCD,
            description="联调测试点云路径（base_link 系，不走坐标变换），受 load_test_cloud 开关控制。"
                        f"当前默认：{DEFAULT_INPUT_PCD or '（无：文件不存在）'}"),
        Node(
            package="rus_sim_perception",
            executable="rus_sim_perception_node",
            name="perception_node",
            output="screen",
            parameters=[params_file, {
                "source": LaunchConfiguration("source"),
                "replay_path": LaunchConfiguration("replay_path"),
                # load_test_cloud=false 或 input_pcd 为空 → 传空串给节点（= 不加载离线点云）
                "input_pcd": PythonExpression([
                    "'' if '", LaunchConfiguration("load_test_cloud"),
                    "'.lower() in ('false', '0', 'no', 'off') or not '",
                    LaunchConfiguration("input_pcd"), "' else '",
                    LaunchConfiguration("input_pcd"), "'",
                ]),
            }],
        ),
    ])
