#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════════════
#  规划轨迹可视化（外部绘制脚本，不污染 planning 代码）
#  ────────────────────────────────────────────────────────────────────
#  订阅 planning 发布的 /planned_trajectory（PoseArray，全部稠密轨迹点），
#  在 RViz 中以美观方式绘制：
#    · LINE_STRIP   —— 绿色轨迹线（把所有点连成一条线）
#    · 圆点层        —— 抽样小点，突出路径走向
#    · 起点/终点标记 —— 红球(起点) / 蓝球(终点)
#  方向（箭头）不重要，不显示。
#
#  用法（系统已启动，planning 在跑）：
#     source install/setup.bash
#     python3 src/rus_sim_bringup/scripts/traj_vis.py
#   RViz：Fixed Frame=base_link，添加 MarkerArray 显示 /trajectory_vis。
# ════════════════════════════════════════════════════════════════════

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, PoseArray
from visualization_msgs.msg import Marker, MarkerArray


class TrajVis(Node):
    def __init__(self):
        super().__init__('traj_vis')
        self.sub = self.create_subscription(
            PoseArray, '/planned_trajectory', self.on_traj, 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, '/trajectory_vis', 10)
        self.get_logger().info('TrajVis 已启动，订阅 /planned_trajectory')

    def on_traj(self, msg):
        pts = msg.poses
        if len(pts) < 2:
            return

        # ── 诊断：打印轨迹 z 统计，判断"到达终点前掉一截"是轨迹本身还是执行 ──
        zs = [p.position.z for p in pts]
        z_min = min(zs)
        i_min = zs.index(z_min)
        # 相邻点最大跳跃（抖动诊断）
        dmax, idmax = 0.0, 1
        for i in range(1, len(pts)):
            a, b = pts[i - 1].position, pts[i].position
            d = ((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2) ** 0.5
            if d > dmax:
                dmax, idmax = d, i
        self.get_logger().info(
            '轨迹 %d 点 | z∈[%.3f, %.3f] | z 最低在 %.1f%% 处 (idx %d)'
            % (len(pts), z_min, max(zs), 100.0 * i_min / (len(pts) - 1), i_min))
        self.get_logger().info(
            '  相邻点最大跳跃=%.4f m @%.1f%%（点云表面 z∈[-0.07, 0.03]，若 z< -0.07 说明轨迹陷入表面下方）'
            % (dmax, 100.0 * idmax / (len(pts) - 1)))

        now = self.get_clock().now().to_msg()
        frame = msg.header.frame_id if msg.header.frame_id else 'base_link'

        markers = []

        # ── 1) 轨迹线（LINE_STRIP，绿色）──
        line = Marker()
        line.header.stamp = now
        line.header.frame_id = frame
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.id = 0
        line.scale.x = 0.008                       # 线宽
        line.color.r, line.color.g, line.color.b = 0.12, 0.85, 0.20
        line.color.a = 1.0
        line.points = [Point(x=p.position.x, y=p.position.y, z=p.position.z)
                       for p in pts]
        markers.append(line)

        # ── 2) 圆点层（抽样小点，突出路径走向；黄绿色）──
        dots = Marker()
        dots.header.stamp = now
        dots.header.frame_id = frame
        dots.type = Marker.SPHERE_LIST
        dots.action = Marker.ADD
        dots.id = 1
        dots.scale.x = dots.scale.y = dots.scale.z = 0.012
        dots.color.r, dots.color.g, dots.color.b = 0.9, 0.9, 0.15
        dots.color.a = 0.9
        step = max(1, len(pts) // 60)
        dots.points = [Point(x=pts[i].position.x, y=pts[i].position.y,
                             z=pts[i].position.z) for i in range(0, len(pts), step)]
        markers.append(dots)

        # ── 3) 起点（红球）──
        start = Marker()
        start.header.stamp = now
        start.header.frame_id = frame
        start.type = Marker.SPHERE
        start.action = Marker.ADD
        start.id = 2
        start.scale.x = start.scale.y = start.scale.z = 0.03
        start.color.r, start.color.g, start.color.b = 0.95, 0.15, 0.15
        start.color.a = 1.0
        start.pose.position = pts[0].position
        markers.append(start)

        # ── 4) 终点（蓝球）──
        goal = Marker()
        goal.header.stamp = now
        goal.header.frame_id = frame
        goal.type = Marker.SPHERE
        goal.action = Marker.ADD
        goal.id = 3
        goal.scale.x = goal.scale.y = goal.scale.z = 0.03
        goal.color.r, goal.color.g, goal.color.b = 0.15, 0.45, 0.95
        goal.color.a = 1.0
        goal.pose.position = pts[-1].position
        markers.append(goal)

        self.marker_pub.publish(MarkerArray(markers=markers))


def main():
    rclpy.init()
    node = TrajVis()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
