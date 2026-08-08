import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('rus_sim_data')
    params_file = os.path.join(pkg_dir, 'config', 'data_params.yaml')

    return LaunchDescription([

        # ── 数据节点（录制 / 回放统一管理）──
        Node(
            package='rus_sim_data',
            executable='rus_sim_data_node',
            name='data_node',
            parameters=[params_file],
        ),
    ])
