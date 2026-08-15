#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════════════
#  单目标点 movel 执行验证
#  ────────────────────────────────────────────────────────────────────
#  只发一个 movel 目标（位置 + RPY=[pi,0,0] z 轴朝下压表面），等待完成，
#  然后对比：
#    · 目标位姿（TCP/探头尖）
#    · 实际 flange_pos（驱动发布的 TCP 位置）
#    · 偏差（欧氏距离 + x/y/z 分量）
#    · 实际关节角 joint_pos
#  并给出结论（到位 / 未到位）。
#
#  说明：flange_pos 是「探头尖端（TCP）」位置（驱动内部 = 法兰本体 +
#  flange_offset×z轴）。姿态 z 朝下时，法兰本体在 TCP 上方 0.0938m，
#  这是探头长度，属正常现象。
#
#  用法（系统已启动，driver 在跑）：
#     source install/setup.bash
#     python3 src/rus_sim_bringup/scripts/movel_single_test.py
#     python3 src/rus_sim_bringup/scripts/movel_single_test.py --x -0.4 --y -0.2 --z -0.011
# ════════════════════════════════════════════════════════════════════

import argparse
import math
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, PoseArray

from rus_sim_interfaces.msg import RobotState
from rus_sim_interfaces.srv import CommandService

FLANGE_OFFSET = 0.0938   # 法兰 → 探头尖 TCP 的 Z 向偏移 [m]


def load_cloud(path):
    """解析 PCD 二进制（每点 32 字节），返回 Nx3"""
    with open(path, 'rb') as f:
        raw = f.read()
    idx = raw.find(b'DATA binary\n')
    header = raw[:idx].decode()
    points = int([l.split()[1] for l in header.splitlines()
                  if l.startswith('POINTS')][0])
    data = raw[idx + len(b'DATA binary\n'):]
    return np.frombuffer(data[:points * 32], dtype=np.float32).reshape(points, 8)[:, :3]


class MovelSingleTest(Node):
    def __init__(self, target):
        super().__init__('movel_single_test')
        self.cmd_client = self.create_client(CommandService, '/driver/command')
        self.target_pub = self.create_publisher(PoseArray, '/movel_single_target', 10)
        self.state_sub = self.create_subscription(
            RobotState, '/driver/state', self.on_state, 10)
        self.latest = None          # 最新 state
        self.target = target        # (x, y, z)

    def on_state(self, msg):
        self.latest = msg

    def call(self, cmd, args):
        req = CommandService.Request()
        req.client_id = 0
        req.command = cmd
        req.args = args
        if not self.cmd_client.service_is_ready():
            self.get_logger().warn('驱动服务不可用')
            return None
        return self.cmd_client.call(req)

    def wait_done(self, timeout=15.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            resp = self.call('is_motion_done', [])
            if resp and resp.success and resp.result and resp.result[0] > 0.5:
                return time.time() - t0, True
            time.sleep(0.05)
        return time.time() - t0, False

    def run(self):
        while not self.cmd_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('等待 /driver/command ...')
        self.get_logger().info('驱动服务就绪')

        x, y, z = self.target

        # 发布目标点（RViz 红色箭头）
        arr = PoseArray()
        arr.header.frame_id = 'base_link'
        arr.header.stamp = self.get_clock().now().to_msg()
        p = Pose()
        p.position.x = x
        p.position.y = y
        p.position.z = z
        p.orientation.x = 1.0   # RPY=[pi,0,0] → z 轴朝 -z（向下压表面）
        arr.poses = [p]
        self.target_pub.publish(arr)

        # 运动前状态
        time.sleep(0.2)
        q0 = list(self.latest.joint_pos) if self.latest else None

        self.get_logger().info('')
        self.get_logger().info('═' * 60)
        self.get_logger().info('movel 目标: 位置 (%.4f, %.4f, %.4f)  RPY=(pi, 0, 0)' % (x, y, z))
        self.get_logger().info('═' * 60)
        if q0:
            self.get_logger().info('运动前 q: ' + ' '.join('%.4f' % v for v in q0))

        resp = self.call('movel', [x, y, z, math.pi, 0.0, 0.0])
        if resp is None or not resp.success:
            self.get_logger().error('movel 下发失败')
            return
        self.get_logger().info('movel 已下发，等待 is_motion_done ...')

        elapsed, done = self.wait_done()
        self.get_logger().info('is_motion_done=%s  耗时=%.2f s' % (done, elapsed))

        # 等状态稳定后采样（取最近 1s 内的最后一帧，避免读到运动中间态）
        time.sleep(0.5)
        state = None
        for _ in range(20):
            if self.latest is not None:
                state = self.latest
            time.sleep(0.05)

        if state is None:
            self.get_logger().error('未收到 /driver/state')
            return

        fx, fy, fz = state.flange_pos[0], state.flange_pos[1], state.flange_pos[2]
        q1 = list(state.joint_pos)

        dx = fx - x
        dy = fy - y
        dz = fz - z
        err = math.sqrt(dx * dx + dy * dy + dz * dz)

        self.get_logger().info('')
        self.get_logger().info('─' * 60)
        self.get_logger().info('实际 flange_pos (TCP): (%.4f, %.4f, %.4f)' % (fx, fy, fz))
        self.get_logger().info('目标 (TCP)          : (%.4f, %.4f, %.4f)' % (x, y, z))
        self.get_logger().info('偏差  Δx=%.4f  Δy=%.4f  Δz=%.4f  |Δ|=%.4f m'
                               % (dx, dy, dz, err))
        self.get_logger().info('实际 q: ' + ' '.join('%.4f' % v for v in q1))
        if q0:
            dq = [a - b for a, b in zip(q1, q0)]
            self.get_logger().info('Δq   : ' + ' '.join('%+.4f' % v for v in dq))
        self.get_logger().info('─' * 60)

        if err < 0.01:
            self.get_logger().info('★ 结论：movel 到位（偏差 < 1cm）')
        else:
            self.get_logger().info('★ 结论：movel 未到位（偏差 %.3f m）' % err)
            self.get_logger().info('  注意：flange_pos=TCP（探头尖）。若偏差≈0.09 说明读到的是法兰本体，'
                                   '请确认 driver 已按当前源码重建。')
        self.get_logger().info('─' * 60)


def main():
    parser = argparse.ArgumentParser(description='单目标点 movel 执行验证')
    parser.add_argument('--x', type=float, default=None)
    parser.add_argument('--y', type=float, default=None)
    parser.add_argument('--z', type=float, default=None)
    args = parser.parse_args()

    if args.x is None or args.y is None or args.z is None:
        cloud = load_cloud('/tmp/test_cloud.pcd')
        # 默认取表面点：x 固定 -0.4，y=-0.2 最近表面点
        probe = np.array([-0.4, -0.2, 0.0])
        d = np.linalg.norm(cloud - probe, axis=1)
        nn = cloud[d.argmin()]
        x, y, z = float(nn[0]), float(nn[1]), float(nn[2])
    else:
        x, y, z = args.x, args.y, args.z

    rclpy.init()
    node = MovelSingleTest((x, y, z))
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

