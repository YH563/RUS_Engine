#!/usr/bin/env python3
"""RUS_Sim 整系统启动：桥接层 + 驱动层 + 规划层。

调用各包内部 launch：
  - rus_sim_bridge/launch/bridge.launch.py        （WS 网关）
  - rus_sim_driver/launch/driver.launch.py        （驱动 + robot_state_publisher + RViz）
  - rus_sim_planning/launch/planning.launch.py    （轨迹规划 / 伺服执行）

用法：
  ros2 launch rus_sim_bringup rus_sim.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


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

    return LaunchDescription([
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
    ])

