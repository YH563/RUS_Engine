#!/usr/bin/env python3
"""
键盘控制机器人点动运动

模式切换（数字键）：
  0 — 关节空间点动 (JOG_0)      1 — 基坐标系点动 (JOG_1)
  2 — 工具坐标系点动 (JOG_2)

按键映射（JOG_0 关节1~6 / JOG_1&2 X Y Z Rx Ry Rz）：
  Q/A 轴1   W/S 轴2   E/D 轴3
  R/F 轴4   T/G 轴5   Y/H 轴6

通用：
  Esc        — 退出

原理：按下按键 → start_jog 启动连续运动
      松开按键（检测到无输入超时）→ stop_jog_decel 减速停止
      连续按压同键不做重复请求，点动段持续运行。
"""

import sys
import tty
import termios
import select
import threading

import time

import rclpy
from rclpy.node import Node
from rus_sim_driver.srv import CommandService

# 按键 → (joint_nb, dir)  nb=1~6，各模式通用
JOG_KEYS = {
    'q': (1, 1),   'a': (1, 0),     # 轴 1
    'w': (2, 1),   's': (2, 0),     # 轴 2
    'e': (3, 1),   'd': (3, 0),     # 轴 3
    'r': (4, 1),   'f': (4, 0),     # 轴 4
    't': (5, 1),   'g': (5, 0),     # 轴 5
    'y': (6, 1),   'h': (6, 0),     # 轴 6
}

JOG_SPEED = 30    # 速度百分比（30% × 0.5 rad/s ≈ 0.15 rad/s ≈ 8.6°/s）
JOG_ACC   = 30    # 加速度百分比
TIMEOUT   = 0.20  # 按键释放检测超时 [s]

# 点动模式：mode_id → {名称, ref 值, 轴标签}
JOG_MODES = {
    0: {'name': '关节空间 (JOG_0)', 'ref': 0,  'labels': ['关节1', '关节2', '关节3', '关节4', '关节5', '关节6']},
    1: {'name': '基坐标系 (JOG_1)', 'ref': 2,  'labels': ['X',    'Y',    'Z',    'Rx',   'Ry',   'Rz']},
    2: {'name': '工具坐标系 (JOG_2)', 'ref': 4, 'labels': ['X',    'Y',    'Z',    'Rx',   'Ry',   'Rz']},
}


class KeyboardController(Node):
    def __init__(self):
        super().__init__('keyboard_controller')

        # ── 服务客户端 ──
        self.cli = self.create_client(CommandService, '/driver/command')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待 /driver/command 服务...')

        self.running = True
        self.active_key = None       # 当前按下的键
        self.last_key_time = 0.0     # 上一次按键时间
        self.current_mode = 0        # 当前点动模式 0/1/2

        # 按键队列（键盘读取线程 → 主线程）
        self.key_queue = []
        self.key_queue_lock = threading.Lock()

    def _call(self, cmd, args=None):
        """异步调用服务"""
        req = CommandService.Request()
        req.command = cmd
        req.args = args or []
        self.cli.call_async(req)

    def start_jog(self, key):
        """启动点动"""
        if key not in JOG_KEYS:
            return
        mode = JOG_MODES[self.current_mode]
        nb, direction = JOG_KEYS[key]
        args = [float(mode['ref']), float(nb), float(direction),
                float(JOG_SPEED), float(JOG_ACC), 0.0]
        self._call('start_jog', args)
        self.active_key = key

        label = mode['labels'][nb - 1]
        dir_str = '+' if direction == 1 else '-'
        print(f'\r  [{mode["name"]}] {label}{dir_str}  0/1/2切换  Esc退出  ', end='', flush=True)

    def stop_jog(self):
        """减速停止当前点动"""
        if self.active_key is None:
            return
        req = CommandService.Request()
        req.command = 'stop_jog_decel'
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        self.active_key = None

    def stop_jog_immediate(self):
        """立即停止当前点动"""
        if self.active_key is None:
            return
        req = CommandService.Request()
        req.command = 'stop_jog_immediate'
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        self.active_key = None

    def handle_key(self, key):
        now = time.monotonic()
        self.last_key_time = now

        # ── 退出 ──
        if key == '\x1b':
            self.stop_jog_immediate()
            self.running = False
            return

        # ── 模式切换（0/1/2）：立即停止旧段，新模式下一次按键生效 ──
        if key in ('0', '1', '2'):
            new_mode = int(key)
            if new_mode != self.current_mode:
                self.stop_jog_immediate()
                self.current_mode = new_mode
                mode = JOG_MODES[new_mode]
                print(f'\r  切换到 {mode["name"]}  ', end='', flush=True)
            return

        # ── 点动键 ──
        if key in JOG_KEYS:
            if key != self.active_key:
                self.stop_jog_immediate()
                self.start_jog(key)
            else:
                self.last_key_time = now
        else:
            self.stop_jog()


def keyboard_reader(ctrl):
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        while ctrl.running:
            if select.select([sys.stdin], [], [], 0.05)[0]:
                key = sys.stdin.read(1)
                with ctrl.key_queue_lock:
                    # 只在与队尾不同时才入队，防止堆积
                    if not ctrl.key_queue or ctrl.key_queue[-1] != key:
                        ctrl.key_queue.append(key)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main():
    rclpy.init()
    ctrl = KeyboardController()

    print('=== 键盘控制 — 点动运动 ===')
    print('模式: 0=关节空间  1=基坐标系  2=工具坐标系')
    print('Q/A 轴1  W/S 轴2  E/D 轴3')
    print('R/F 轴4  T/G 轴5  Y/H 轴6')
    print('按住运动，松开停止 | Esc: 退出')
    print(f'当前模式: {JOG_MODES[ctrl.current_mode]["name"]}')
    print()

    reader_thread = threading.Thread(target=keyboard_reader, args=(ctrl,), daemon=True)
    reader_thread.start()

    while ctrl.running:
        # 处理按键
        key = None
        with ctrl.key_queue_lock:
            if ctrl.key_queue:
                key = ctrl.key_queue.pop(0)
        if key is not None:
            ctrl.handle_key(key)

        # 检测按键释放：超时无输入（用单调时钟，不依赖 ROS 时间）
        if ctrl.active_key is not None and (time.monotonic() - ctrl.last_key_time) > TIMEOUT:
            ctrl.stop_jog()

        rclpy.spin_once(ctrl, timeout_sec=0.01)

    ctrl.stop_jog()
    ctrl.destroy_node()
    rclpy.shutdown()
    print('\n退出')


if __name__ == '__main__':
    main()
