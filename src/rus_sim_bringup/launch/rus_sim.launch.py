#!/usr/bin/env python3
"""RUS_Sim 整系统启动：桥接层 + 驱动层 + 规划层 + 感知层（+ 可选记录层）。

调用各包内部 launch：
  - rus_sim_bridge/launch/bridge.launch.py        （WS 网关）
  - rus_sim_driver/launch/driver.launch.py        （驱动 + robot_state_publisher + RViz）
  - rus_sim_planning/launch/planning.launch.py    （轨迹规划 / 伺服执行）
  - rus_sim_perception/launch/perception.launch.py（实时建图 / 离线点云加载）
  - rus_sim_recorder/launch/recorder.launch.py    （记录层，仅 record:=true 时拉起）

用法：
  ros2 launch rus_sim_bringup rus_sim.launch.py
  ros2 launch rus_sim_bringup rus_sim.launch.py record:=true              # 同时录制
  ros2 launch rus_sim_bringup rus_sim.launch.py record:=true record_dir:=/data/run01
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


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
        # 桥接层：前后端 WS 网关（/control /state /sensor）
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bridge_launch),
        ),
        # 驱动层：驱动节点 + robot_state_publisher + RViz
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(driver_launch),
        ),
        # 规划层：轨迹生成 / 插值 / 伺服执行
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(planning_launch),
        ),
        # 感知层：实时建图 / 离线点云加载
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(perception_launch),
        ),
        # 记录层（可选）：/driver/state + /sensor/pointcloud → <record_dir>/*.rusrec
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(recorder_launch),
            launch_arguments={'output_dir': LaunchConfiguration('record_dir')}.items(),
            condition=IfCondition(LaunchConfiguration('record')),
        ),
    ])

