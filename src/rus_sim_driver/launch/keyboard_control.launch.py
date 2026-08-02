import os
from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    install_prefix = get_package_prefix('rus_sim_driver')
    script_path = os.path.join(install_prefix, 'lib', 'rus_sim_driver', 'keyboard_control.py')

    # 机器人SDK工作空间根目录（install 的父目录）
    ws_root = os.path.abspath(os.path.join(install_prefix, '..', '..'))

    # keyboard_control.py 使用 termios/tty 读取原始键盘输入，
    # 需要在一个真正的终端窗口中运行，因此用 gnome-terminal 包裹。
    # bash -ic 会加载 .bashrc（含 ROS2 source），
    # cd 到工作空间目录触发自动 source install/setup.bash。
    return LaunchDescription([

        ExecuteProcess(
            cmd=['gnome-terminal', '--',
                 'bash', '-ic',
                 f'cd {ws_root} && python3 {script_path}; exec bash'],
            name='keyboard_control',
            output='screen',
        ),

    ])
