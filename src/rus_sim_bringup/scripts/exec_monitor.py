#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════════════
#  伺服执行监控 v2：记录实际 TCP 位置/姿态 vs 规划轨迹
#  ────────────────────────────────────────────────────────────────────
#  定位"接近终点 TCP 冲高/掉"：记录实际 flange_pos（TCP + RPY），
#  scan_done 时打印：
#     · 实际 TCP z 全曲线（按执行比例采样）vs 轨迹 z
#     · 接近终点（最后 20%）的 z 与 RPY 变化（判断末端姿态是否翻转）
# ════════════════════════════════════════════════════════════════════

import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray
from rus_sim_interfaces.msg import RobotState, ModuleEvent


class ExecMonitor(Node):
    def __init__(self):
        super().__init__('exec_monitor')
        self.state_sub = self.create_subscription(
            RobotState, '/driver/state', self.on_state, 100)
        self.traj_sub = self.create_subscription(
            PoseArray, '/planned_trajectory', self.on_traj, 10)
        self.evt_sub = self.create_subscription(
            ModuleEvent, '/module_events', self.on_event, 10)
        self.traj = None
        self.actual = []          # [z, rx, ry, rz]
        self.get_logger().info('ExecMonitor 已启动')

    def on_state(self, msg):
        if len(msg.flange_pos) >= 6:
            self.actual.append([msg.flange_pos[2],
                                msg.flange_pos[3], msg.flange_pos[4], msg.flange_pos[5]])

    def on_traj(self, msg):
        self.traj = [(p.position.x, p.position.y, p.position.z) for p in msg.poses]
        self.actual = []
        zs = [p[2] for p in self.traj]
        self.get_logger().info('已记录规划轨迹 %d 点 z∈[%.3f, %.3f]' % (len(self.traj), min(zs), max(zs)))

    def on_event(self, msg):
        if msg.event == 'scan_done':
            self.report()

    def report(self):
        self.get_logger().info('═' * 60)
        self.get_logger().info('执行结束统计')
        self.get_logger().info('═' * 60)

        if not self.traj or len(self.actual) < 2:
            self.get_logger().warn('数据不足（需要先 plan + execute）')
            return

        tz = [p[2] for p in self.traj]
        az = [a[0] for a in self.actual]
        n = len(self.actual)

        self.get_logger().info('轨迹 z     : [%.4f, %.4f]' % (min(tz), max(tz)))
        self.get_logger().info('实际 TCP z : [%.4f, %.4f]   (轨迹终点 z=%.4f)'
                               % (min(az), max(az), tz[-1]))
        self.get_logger().info('  超出轨迹上界 %.4f m' % (max(az) - max(tz)))

        # 实际 z 曲线按执行比例采样
        self.get_logger().info('实际 z 曲线采样：')
        marks = [0.0, 0.2, 0.4, 0.6, 0.8, 0.9, 0.95, 1.0]
        for f in marks:
            idx = min(int(f * (n - 1)), n - 1)
            z, rx, ry, rz = self.actual[idx]
            self.get_logger().info(
                '  %3.0f%%  z=%.4f  RPY=(%.2f, %.2f, %.2f)' % (f * 100, z, rx, ry, rz))

        # 接近终点（最后 20%）冲高定位
        tail = self.actual[int(n * 0.8):]
        z_tail = [a[0] for a in tail]
        i_max = z_tail.index(max(z_tail))
        self.get_logger().info('接近终点（最后 20%%）TCP z 最高 %.4f（轨迹终点 %.4f，超 %.4f）'
                               % (max(z_tail), tz[-1], max(z_tail) - tz[-1]))
        # 冲高时姿态
        self.get_logger().info('  冲高时刻 RPY=(%.2f, %.2f, %.2f)  (rx≈±%.2f 表示 z 轴朝下正常；rx 突变→姿态翻转)'
                               % (tail[i_max][1], tail[i_max][2], tail[i_max][3], math.pi))
        self.get_logger().info('═' * 60)


def main():
    rclpy.init()
    node = ExecMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.report()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
