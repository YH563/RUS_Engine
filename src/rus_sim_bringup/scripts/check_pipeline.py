#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════════════
#  全链路综合检测：点云表面 vs 规划轨迹 vs 实际 TCP
#  ────────────────────────────────────────────────────────────────────
#  对齐三方数据，全采样检测"白球是否掉进点云下方"：
#    · 点云（/preprocessed_cloud）→ 表面高度基准
#    · 规划轨迹（/planned_trajectory）→ 参考轨迹
#    · 实际 TCP（/driver/state flange_pos）→ 全量记录
#   scan_done 或 Ctrl-C 时输出：
#     1) 轨迹是否贴表面（轨迹点相对点云表面的 z 偏差）
#     2) 实际 TCP 是否掉进点云下方（穿透深度、位置、发生时段）
#     3) 实际 TCP 相对轨迹的偏差
#
#  用法（系统已启动）：
#     python3 src/rus_sim_bringup/scripts/check_pipeline.py
#     另开终端跑 test_prescan.sh
# ════════════════════════════════════════════════════════════════════

import math
import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from rus_sim_interfaces.msg import RobotState, ModuleEvent


class CheckPipeline(Node):
    def __init__(self):
        super().__init__('check_pipeline')
        self.cloud_sub = self.create_subscription(
            PointCloud2, '/preprocessed_cloud', self.on_cloud, 10)
        self.traj_sub = self.create_subscription(
            PoseArray, '/planned_trajectory', self.on_traj, 10)
        self.state_sub = self.create_subscription(
            RobotState, '/driver/state', self.on_state, 100)
        self.evt_sub = self.create_subscription(
            ModuleEvent, '/module_events', self.on_event, 10)
        self.cloud = None        # Nx3
        self.traj = None         # [(x,y,z), ...]
        self.traj_poses = None   # 完整轨迹 Pose（含姿态）
        self.actual = []         # [x,y,z,q1..q6, ...]
        self.get_logger().info('CheckPipeline 已启动')

    def on_cloud(self, msg):
        try:
            pts = pc2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
            if pts.size == 0:
                return
            self.cloud = pts.astype(np.float64).reshape(-1, 3)
            self.get_logger().info('点云 %d 点 z∈[%.3f, %.3f]' %
                                   (len(self.cloud), self.cloud[:, 2].min(), self.cloud[:, 2].max()))
        except Exception as e:
            self.get_logger().warn('点云读取失败: %s' % e)

    def on_traj(self, msg):
        self.traj_poses = [p for p in msg.poses]   # 完整 Pose（含姿态）
        self.traj = [(p.position.x, p.position.y, p.position.z) for p in msg.poses]
        self.actual = []
        zs = [p[2] for p in self.traj]
        self.get_logger().info('规划轨迹 %d 点 z∈[%.3f, %.3f]' % (len(self.traj), min(zs), max(zs)))

    def on_state(self, msg):
        if len(msg.flange_pos) >= 3:
            q = list(msg.joint_pos) if len(msg.joint_pos) >= 6 else [0] * 6
            self.actual.append([msg.flange_pos[0], msg.flange_pos[1], msg.flange_pos[2]] + q)

    def on_event(self, msg):
        if msg.event == 'scan_done':
            self.report()

    def _quat_to_rpy(self, p):
        """固定轴 XYZ RPY（与 driver flange_pos[3..5] 一致）"""
        x, y, z, w = p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w
        R = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
        rx = math.atan2(R[2, 1], R[2, 2])
        ry = math.asin(-R[2, 0])
        rz = math.atan2(R[1, 0], R[0, 0])
        return (rx, ry, rz)

    def _surface_z(self, pts):
        """对每个查询点 (x,y)，找点云中 (x,y) 最近点的 z 作为表面高度"""
        q = np.asarray(pts, dtype=np.float64)[:, :2]
        d2 = (self.cloud[:, :2][None, :, :] - q[:, None, :]) ** 2
        idx = d2.sum(axis=2).argmin(axis=1)
        return self.cloud[idx, 2]

    def report(self):
        self.get_logger().info('═' * 62)
        self.get_logger().info('全链路综合检测报告')
        self.get_logger().info('═' * 62)

        if self.cloud is None or len(self.actual) < 2:
            self.get_logger().warn('数据不足（需要点云 + 执行记录）')
            return

        # ---- 1) 轨迹是否贴表面 ----
        self.get_logger().info('① 轨迹贴表面检查')
        if self.traj:
            tp = np.asarray(self.traj)
            surf = self._surface_z(tp)
            dz = tp[:, 2] - surf
            self.get_logger().info('   轨迹点相对点云表面 z 偏差: [%.4f, %.4f] (均 %.4f)'
                                   % (dz.min(), dz.max(), dz.mean()))
            if dz.min() < -0.005:
                self.get_logger().info('   ★ 轨迹本身就有 %.4f m 低于表面 → 轨迹生成问题' % dz.min())
            else:
                self.get_logger().info('   轨迹贴合表面 ✓')

        # ---- 1.5) 轨迹方向突变检测（xy 平面切线方向角变化）----
        self.get_logger().info('①b 轨迹方向突变检测')
        if self.traj and len(self.traj) > 10:
            tp = np.asarray(self.traj)
            d = np.diff(tp, axis=0)
            ang = np.arctan2(d[:, 1], d[:, 0])
            dang = np.abs(np.diff(ang))
            dang = np.minimum(dang, 2 * np.pi - dang)
            imax = int(np.argmax(dang))
            self.get_logger().info('   最大方向变化 %.1f° @ %.1f%% (x=%.3f y=%.3f)'
                                   % (np.degrees(dang[imax]),
                                      100.0 * imax / len(dang), tp[imax, 0], tp[imax, 1]))
            # 方向变化 > 45° 的突变点数量
            n_sharp = int((dang > np.radians(45)).sum())
            self.get_logger().info('   方向突变(>45°)点数: %d 个' % n_sharp)

        # ---- 2) 实际 TCP 是否掉进点云下方 ----
        self.get_logger().info('② 实际 TCP vs 点云表面（掉进下方检测）')
        ap = np.asarray(self.actual)
        surf_a = self._surface_z(ap)
        pen = ap[:, 2] - surf_a          # <0 = 深入表面下方
        n = len(ap)
        self.get_logger().info('   实际 TCP z: [%.4f, %.4f]' % (ap[:, 2].min(), ap[:, 2].max()))
        self.get_logger().info('   相对表面穿透(实际-表面): 最深 %.4f m' % pen.min())
        if pen.min() < -0.005:
            i = int(pen.argmin())
            self.get_logger().info('   ★ 掉进点云下方！最深 %.4f m @ (%.3f, %.3f, %.3f) 发生在 %.1f%% 处'
                                   % (pen.min(), ap[i, 0], ap[i, 1], ap[i, 2],
                                      100.0 * i / (n - 1)))
            below = pen < -0.005
            if below.any():
                start = int(np.argmax(below))
                end = int(n - 1 - np.argmax(below[::-1]))
                self.get_logger().info('   低于表面持续区间: %.1f%% ~ %.1f%%（该段内最深 %.4f）'
                                       % (100.0 * start / (n - 1), 100.0 * end / (n - 1),
                                          pen[start:end + 1].min()))
        else:
            self.get_logger().info('   实际 TCP 未掉进点云下方 ✓')

        # ---- 3) 实际 TCP vs 规划轨迹 ----
        self.get_logger().info('③ 实际 TCP vs 规划轨迹')
        if self.traj:
            tp = np.asarray(self.traj)
            d2 = ((tp[:, :2][None, :, :] - ap[:, :2][:, None, :]) ** 2).sum(axis=2)
            idx = d2.argmin(axis=1)
            dz_t = ap[:, 2] - tp[idx, 2]
            self.get_logger().info('   相对最近轨迹点 z 偏差: [%.4f, %.4f]'
                                   % (dz_t.min(), dz_t.max()))
            if dz_t.min() < -0.005:
                i = int(dz_t.argmin())
                self.get_logger().info('   ★ 实际比轨迹低 %.4f m @ %.1f%% 处（轨迹该处 z=%.4f，实际 %.4f）'
                                       % (dz_t.min(), 100.0 * i / (n - 1),
                                          tp[idx[i], 2], ap[i, 2]))

        # ---- 4) 轨迹末段姿态检查（85-100%）----
        self.get_logger().info('④ 轨迹末段目标姿态（85%%~100%% RPY）')
        if self.traj_poses and len(self.traj_poses) > 10:
            m = len(self.traj_poses)
            marks = [0.85, 0.87, 0.89, 0.91, 0.93, 0.95, 0.98, 1.0]
            for f in marks:
                p = self.traj_poses[min(int(f * (m - 1)), m - 1)]
                rx, ry, rz = self._quat_to_rpy(p)
                self.get_logger().info('   %3.0f%%  pos z=%.4f  RPY=(%.2f, %.2f, %.2f)'
                                       % (f * 100, p.position.z, rx, ry, rz))

        # ---- 5) 实际执行末段（85-100%）z 与 q 变化（检测 IK 解突变 vs 平滑滞后）----
        self.get_logger().info('⑤ 实际执行末段（85%%~100%%）z 与关节 q')
        if n > 20:
            tail_idx = [int(f * (n - 1)) for f in (0.85, 0.87, 0.89, 0.91, 0.93, 0.95, 0.98, 1.0)]
            prev_q = None
            for i in tail_idx:
                z, q = self.actual[i][2], self.actual[i][3:9]
                dq = ''
                if prev_q is not None:
                    dq = ' '.join('%+.3f' % (q[j] - prev_q[j]) for j in range(6))
                self.get_logger().info('   %3.0f%%  z=%.4f  q=(%s)  Δq=(%s)'
                                       % (100.0 * i / (n - 1), z,
                                          ' '.join('%.3f' % v for v in q), dq))
                prev_q = q
        self.get_logger().info('═' * 62)


def main():
    rclpy.init()
    node = CheckPipeline()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.report()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()

