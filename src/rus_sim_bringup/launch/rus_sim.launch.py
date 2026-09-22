#!/usr/bin/env python3
"""RUS_Sim 整系统启动：桥接层 + 驱动层 + 规划层 + 感知层（+ 可选记录层）。

调用各包内部 launch：
  - rus_sim_bridge/launch/bridge.launch.py        （WS 网关）
  - rus_sim_driver/launch/driver.launch.py        （驱动 + robot_state_publisher；RViz 已注释）
  - rus_sim_planning/launch/planning.launch.py    （轨迹规划 / 伺服执行）
  - rus_sim_perception/launch/perception.launch.py（实时建图 / 离线点云加载）
  - rus_sim_recorder/launch/recorder.launch.py    （记录层，仅 record:=true 时拉起）

用法：
  ros2 launch rus_sim_bringup rus_sim.launch.py
  ros2 launch rus_sim_bringup rus_sim.launch.py record:=true              # 同时录制
  ros2 launch rus_sim_bringup rus_sim.launch.py record:=true record_dir:=/data/run01
  ros2 launch rus_sim_bringup rus_sim.launch.py load_test_cloud:=false    # 不加载联调测试点云
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# 联调测试点云默认值：与 rus_sim_perception/launch/perception.launch.py 保持一致，并透传给感知层
TEST_CLOUD_PATH = os.path.expanduser("~/.rus_sim/test_data/test_cloud.pcd")
DEFAULT_INPUT_PCD = TEST_CLOUD_PATH if os.path.isfile(TEST_CLOUD_PATH) else ""


def generate_launch_description():
    bridge_launch = os.path.join(
        get_package_share_directory('rus_sim_bridge'),
        'launch', 'bridge.launch.py')

    driver_launch = os.path.join(
        get_package_share_directory('rus_sim_driver'),
        'launch', 'driver.launch.py')

    planning_launch = os.path.join(
        get_package_share_directory('rus_sim_planning'),
        'launch', 'planning.launch.py')

    perception_launch = os.path.join(
        get_package_share_directory('rus_sim_perception'),
        'launch', 'perception.launch.py')

    recorder_launch = os.path.join(
        get_package_share_directory('rus_sim_recorder'),
        'launch', 'recorder.launch.py')

    return LaunchDescription([
        # 记录层默认关：录制会产生持续磁盘写入（点云通道可达 MB/s 量级）
        DeclareLaunchArgument(
            'record', default_value='false',
            description='是否同时启动 rus_sim_recorder 录制数据流'),
        DeclareLaunchArgument(
            'record_dir', default_value='records',
            description='录制输出目录（相对路径按启动工作目录解析）'),
        # 联调测试点云开关（透传给感知层）：默认加载 ~/.rus_sim/test_data/test_cloud.pcd
        DeclareLaunchArgument(
            'load_test_cloud', default_value='true',
            description='感知层是否在启动时加载联调测试点云；false = 全栈不灌测试点云'),
        DeclareLaunchArgument(
            'input_pcd', default_value=DEFAULT_INPUT_PCD,
            description='联调测试点云路径（base_link 系），受 load_test_cloud 开关控制'),
        # 桥接层：前后端 WS 网关（/control /state /sensor）
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bridge_launch),
        ),
        # 驱动层：驱动节点 + robot_state_publisher（RViz 已注释，见 driver.launch.py）
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(driver_launch),
        ),
        # 规划层：轨迹生成 / 插值 / 伺服执行
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(planning_launch),
        ),
        # 感知层：实时建图 / 离线点云加载（联调测试点云用 load_test_cloud:=false 关闭）
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(perception_launch),
            launch_arguments={
                'load_test_cloud': LaunchConfiguration('load_test_cloud'),
                'input_pcd': LaunchConfiguration('input_pcd'),
            }.items(),
        ),
        # 记录层（可选）：/driver/state + /sensor/pointcloud → <record_dir>/*.rusrec
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(recorder_launch),
            launch_arguments={'output_dir': LaunchConfiguration('record_dir')}.items(),
            condition=IfCondition(LaunchConfiguration('record')),
        ),
    ])

