#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════════════
#  movel 独立测试脚本（不经过 planning）
#  ────────────────────────────────────────────────────────────────────
#  目的：单独验证驱动执行 movel 的效果。
#   1. 从 /tmp/test_cloud.pcd 表面取一串目标点（沿表面一条线，z 取表面高度）
#   2. 逐段发 movel 到 /driver/command（每段等 is_motion_done）
#   3. RViz 显示：
#        /movel_targets  目标点（红色箭头，位置+朝向）
#        /movel_trace    实时运动轨迹（白色，随 /driver/state 累积）
#   配合 flange_vis 看白球/蓝箭头实际法兰位姿。
#
#  用法（系统已启动，driver 在跑）：
#     source install/setup.bash
#     python3 src/rus_sim_bringup/scripts/movel_test.py
# ════════════════════════════════════════════════════════════════════

import math
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, PoseArray

from rus_sim_interfaces.msg import RobotState
from rus_sim_interfaces.srv import CommandService


def load_cloud(path):
    """解析 PCD 二进制（每点 32 字节：x,y,z + pad + rgb + pad），返回 Nx3"""
    with open(path, 'rb') as f:
        raw = f.read()
    idx = raw.find(b'DATA binary\n')
    header = raw[:idx].decode()
    points = int([l.split()[1] for l in header.splitlines()
                  if l.startswith('POINTS')][0])
    data = raw[idx + len(b'DATA binary\n'):]
    return np.frombuffer(data[:points * 32], dtype=np.float32).reshape(points, 8)[:, :3]


class MovelTest(Node):
    def __init__(self):
        super().__init__('movel_test')
        self.cmd_client = self.create_client(CommandService, '/driver/command')
        self.target_pub = self.create_publisher(PoseArray, '/movel_targets', 10)
        self.trace_pub = self.create_publisher(PoseArray, '/movel_trace', 10)
        self.state_sub = self.create_subscription(
            RobotState, '/driver/state', self.on_state, 10)
        self.trace = []          # 实际运动轨迹点
        self.latest_flange = None   # 最新实际法兰位姿 [x,y,z]

    def on_state(self, msg):
        if len(msg.flange_pos) < 3:
            return
        self.latest_flange = [msg.flange_pos[0], msg.flange_pos[1], msg.flange_pos[2]]
        p = Pose()
        p.position.x = msg.flange_pos[0]
        p.position.y = msg.flange_pos[1]
        p.position.z = msg.flange_pos[2]
        p.orientation.w = 1.0
        self.trace.append(p)
        if len(self.trace) > 4000:
            self.trace = self.trace[-2000:]
        arr = PoseArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.header.frame_id = 'base_link'
        step = max(1, len(self.trace) // 200)
        arr.poses = self.trace[::step]
        self.trace_pub.publish(arr)

    def call(self, cmd, args):
        req = CommandService.Request()
        req.client_id = 0
        req.command = cmd
        req.args = args
        if not self.cmd_client.service_is_ready():
            self.get_logger().warn('驱动服务不可用')
            return None
        return self.cmd_client.call(req)

    def wait_done(self, timeout=10.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            resp = self.call('is_motion_done', [])
            if resp and resp.success and resp.result and resp.result[0] > 0.5:
                return True
            time.sleep(0.1)
        return False

    def run(self):
        while not self.cmd_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('等待 /driver/command ...')
        self.get_logger().info('驱动服务就绪')

        cloud = load_cloud('/tmp/test_cloud.pcd')
        self.get_logger().info('点云 %d 点' % len(cloud))

        # 沿表面一条线生成 movel 目标点（x=-0.4，y 扫掠，z 取表面最近点）
        targets = []
        x_line = -0.4
        for y in np.arange(-0.2, 0.21, 0.02):
            d = np.linalg.norm(cloud - np.array([x_line, y, 0.0]), axis=1)
            nn = cloud[d.argmin()]
            pose = Pose()
            pose.position.x = float(nn[0])
            pose.position.y = float(nn[1])
            pose.position.z = float(nn[2])
            # 姿态：z 轴朝 -z（朝下压表面）。固定轴 XYZ RPY=[pi,0,0] → 四元数 (1,0,0,0)
            pose.orientation.x = 1.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = 0.0
            targets.append(pose)

        # 发布目标点（RViz 红色箭头）
        arr = PoseArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.header.frame_id = 'base_link'
        arr.poses = targets
        self.target_pub.publish(arr)
        self.get_logger().info('movel 目标点 %d 个已发布到 /movel_targets' % len(targets))

        # 逐段发 movel
        for i, t in enumerate(targets):
            self.get_logger().info('movel[%d/%d] → (%.3f, %.3f, %.3f)'
                                   % (i + 1, len(targets),
                                      t.position.x, t.position.y, t.position.z))
            resp = self.call('movel', [t.position.x, t.position.y, t.position.z,
                                       math.pi, 0.0, 0.0])
            if resp is None or not resp.success:
                self.get_logger().warn('movel 失败')
                break
            if not self.wait_done(10.0):
                self.get_logger().warn('movel 超时')
            # 到位对比：实际 flange_pos vs movel 目标
            time.sleep(0.2)   # 等状态刷新
            if self.latest_flange is not None:
                dx = self.latest_flange[0] - t.position.x
                dy = self.latest_flange[1] - t.position.y
                dz = self.latest_flange[2] - t.position.z
                err = math.sqrt(dx * dx + dy * dy + dz * dz)
                self.get_logger().info(
                    '  实际 (%.3f, %.3f, %.3f) vs 目标 (%.3f, %.3f, %.3f)  偏差=%.4f m'
                    % (self.latest_flange[0], self.latest_flange[1], self.latest_flange[2],
                       t.position.x, t.position.y, t.position.z, err))
        self.get_logger().info('movel 测试完成')


def main():
    rclpy.init()
    node = MovelTest()
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
