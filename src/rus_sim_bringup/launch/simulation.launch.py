"""RUS_Sim 整系统启动：桥接层 + 驱动层。

调用各包内部 launch：
  - rus_sim_bridge/launch/bridge.launch.py        （WS 网关）
  - rus_sim_driver/launch/simulation.launch.py    （驱动 + robot_state_publisher + RViz）

用法：
  ros2 launch rus_sim_bringup simulation.launch.py
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
        'launch', 'simulation.launch.py')

    return LaunchDescription([
        # 桥接层：前后端 WS 网关（/control /state /sensor）
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bridge_launch),
        ),
        # 驱动层：驱动节点 + robot_state_publisher + RViz
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(driver_launch),
        ),
    ])
