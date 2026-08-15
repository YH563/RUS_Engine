#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════════════
#  法兰位姿实时可视化（调试用）
#  ────────────────────────────────────────────────────────────────────
#  订阅 /driver/state（RobotState.flange_pos [x,y,z,rx,ry,rz]，
#  固定轴 XYZ RPY，R = Rz·Ry·Rx），在 RViz 中显示：
#    · TF 坐标系  base_link → flange_vis（XYZ 三轴，判断位置与朝向）
#    · Marker    球（法兰位置）+ 蓝色箭头（法兰 z 轴方向，探头朝向）
#
#  用法：
#     source install/setup.bash
#     python3 src/rus_sim_bringup/scripts/flange_vis.py
#   RViz：Fixed Frame=base_link，添加 TF 显示；添加 Marker 显示 /flange_marker。
# ════════════════════════════════════════════════════════════════════

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, TransformStamped
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from rus_sim_interfaces.msg import RobotState


def mat_mult(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)]
            for i in range(3)]


def rpy_to_rotation(rx, ry, rz):
    """固定轴 XYZ：R = Rz·Ry·Rx（与 RusUtils::FlangePosToPose 一致）"""
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    Rx = [[1, 0, 0], [0, cx, -sx], [0, sx, cx]]
    Ry = [[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]]
    Rz = [[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]]
    return mat_mult(Rz, mat_mult(Ry, Rx))


def quat_mult(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)


def axis_quat(axis, ang):
    x, y, z = axis
    h = ang / 2.0
    s = math.sin(h)
    return (x * s, y * s, z * s, math.cos(h))


def rpy_to_quat(rx, ry, rz):
    """R = Rz·Ry·Rx → q = qz * qy * qx"""
    return quat_mult(quat_mult(axis_quat((0, 0, 1), rz),
                               axis_quat((0, 1, 0), ry)),
                     axis_quat((1, 0, 0), rx))


class FlangeVis(Node):
    def __init__(self):
        super().__init__('flange_vis')
        self.sub = self.create_subscription(
            RobotState, '/driver/state', self.on_state, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.marker_pub = self.create_publisher(MarkerArray, '/flange_marker', 10)
        self.get_logger().info('FlangeVis 已启动，订阅 /driver/state')

    def on_state(self, msg):
        if len(msg.flange_pos) < 6:
            return
        x, y, z = msg.flange_pos[0], msg.flange_pos[1], msg.flange_pos[2]
        rx, ry, rz = msg.flange_pos[3], msg.flange_pos[4], msg.flange_pos[5]

        q = rpy_to_quat(rx, ry, rz)
        R = rpy_to_rotation(rx, ry, rz)
        z_axis = (R[0][2], R[1][2], R[2][2])  # 法兰 z 轴方向（世界系）

        now = self.get_clock().now().to_msg()

        # ── 1) TF 坐标系 base_link → flange_vis ──
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'flange_vis'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

        # ── 2) Marker：球（位置）+ 蓝色箭头（z 轴朝向）──
        sphere = Marker()
        sphere.header.stamp = now
        sphere.header.frame_id = 'base_link'
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.id = 0
        sphere.pose.position.x = x
        sphere.pose.position.y = y
        sphere.pose.position.z = z
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.05
        sphere.color.r = 1.0
        sphere.color.g = 1.0
        sphere.color.b = 1.0
        sphere.color.a = 1.0

        arrow = Marker()
        arrow.header.stamp = now
        arrow.header.frame_id = 'base_link'
        arrow.id = 1
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.points = [
            Point(x=x, y=y, z=z),
            Point(x=x + z_axis[0] * 0.12,
                  y=y + z_axis[1] * 0.12,
                  z=z + z_axis[2] * 0.12),
        ]
        arrow.scale.x = 0.008   # 箭杆直径
        arrow.scale.y = 0.02    # 箭头直径
        arrow.color.r = 0.0
        arrow.color.g = 0.0
        arrow.color.b = 1.0
        arrow.color.a = 1.0

        # MarkerArray 发布（RViz MarkerArray 显示项兼容）
        arr = MarkerArray()
        arr.markers = [sphere, arrow]
        self.marker_pub.publish(arr)

        self.get_logger().debug(
            'flange pos=(%.3f, %.3f, %.3f) rpy=(%.3f, %.3f, %.3f) z_axis=(%.3f, %.3f, %.3f)'
            % (x, y, z, rx, ry, rz, z_axis[0], z_axis[1], z_axis[2]))


def main():
    rclpy.init()
    rclpy.spin(FlangeVis())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
