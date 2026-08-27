#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════════════
#  传感器坐标系实时可视化（法兰 / 相机 / 工具，调试用）
#  ────────────────────────────────────────────────────────────────────
#  订阅 /driver/state（RobotState），在 RViz 中显示：
#    · TF  base_link → flange       法兰坐标系（flange_pos，XYZABC m/rad）
#    · TF  base_link → camera_link  相机坐标系（法兰 × 相机相对法兰矩阵）
#    · TF  base_link → tool_tcp     当前工具坐标系（tool_pose，可选 --no-tool）
#    · TF  flange    → tool_calib   六点标定结果（TCP 相对法兰，可选 --tool-calib）
#    · Marker 球（法兰位置）+ 蓝色箭头（法兰 z 轴方向，探头朝向）
#
#  相机相对法兰矩阵默认取实测标定值（4x4 行优先，camera in flange）：
#   T_camera_in_base = T_flange × camera_to_flange
#
#  用法：
#     source install/setup.bash
#     python3 src/rus_sim_bringup/scripts/flange_vis.py
#     python3 src/rus_sim_bringup/scripts/flange_vis.py --camera 1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1
#   RViz：Fixed Frame=base_link，添加 TF 显示；添加 Marker 显示 /flange_marker。
# ════════════════════════════════════════════════════════════════════

import argparse
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, TransformStamped
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from rus_sim_interfaces.msg import RobotState


# 相机相对法兰变换矩阵（4x4 行优先，实测标定值）
# 平移: (0.0938, -0.0357, -0.1361) m；姿态为上方旋转矩阵
DEFAULT_CAMERA_TO_FLANGE = [
    -0.6890016, 0.72474778, -0.00417675, 0.09376472,
    -0.72475632, -0.68896894, 0.00707673, -0.03569825,
    0.0022512, 0.00790301, 0.99996624, -0.13611331,
    0.0, 0.0, 0.0, 1.0,
]


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


def matrix_list_to_4x4(v):
    """16 元素列表（行优先）→ 4x4 嵌套列表"""
    return [[v[i * 4 + j] for j in range(4)] for i in range(4)]


def mat_mult4(A, B):
    """4x4 矩阵乘法"""
    return [[sum(A[i][k] * B[k][j] for k in range(4)) for j in range(4)]
            for i in range(4)]


def quat_from_matrix(R):
    """3x3 旋转矩阵 → 四元数 (x, y, z, w)"""
    trace = R[0][0] + R[1][1] + R[2][2]
    if trace > 0:
        s = 2.0 * math.sqrt(trace + 1.0)
        w = 0.25 * s
        x = (R[2][1] - R[1][2]) / s
        y = (R[0][2] - R[2][0]) / s
        z = (R[1][0] - R[0][1]) / s
    elif R[0][0] > R[1][1] and R[0][0] > R[2][2]:
        s = 2.0 * math.sqrt(1.0 + R[0][0] - R[1][1] - R[2][2])
        w = (R[2][1] - R[1][2]) / s
        x = 0.25 * s
        y = (R[0][1] + R[1][0]) / s
        z = (R[0][2] + R[2][0]) / s
    elif R[1][1] > R[2][2]:
        s = 2.0 * math.sqrt(1.0 + R[1][1] - R[0][0] - R[2][2])
        w = (R[0][2] - R[2][0]) / s
        x = (R[0][1] + R[1][0]) / s
        y = 0.25 * s
        z = (R[1][2] + R[2][1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + R[2][2] - R[0][0] - R[1][1])
        w = (R[1][0] - R[0][1]) / s
        x = (R[0][2] + R[2][0]) / s
        y = (R[1][2] + R[2][1]) / s
        z = 0.25 * s
    return (x, y, z, w)


def matrix_to_tf(parent, child, M):
    """4x4 矩阵 → TransformStamped"""
    t = TransformStamped()
    t.header.frame_id = parent
    t.child_frame_id = child
    t.transform.translation.x = M[0][3]
    t.transform.translation.y = M[1][3]
    t.transform.translation.z = M[2][3]
    q = quat_from_matrix([[M[i][j] for j in range(3)] for i in range(3)])
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]
    return t


def pose_to_tf(parent, child, pose6):
    """[x,y,z,rx,ry,rz]（固定轴 XYZ RPY）→ TransformStamped"""
    x, y, z, rx, ry, rz = pose6[0], pose6[1], pose6[2], pose6[3], pose6[4], pose6[5]
    R = rpy_to_rotation(rx, ry, rz)
    M = [[R[0][0], R[0][1], R[0][2], x],
         [R[1][0], R[1][1], R[1][2], y],
         [R[2][0], R[2][1], R[2][2], z],
         [0, 0, 0, 1]]
    return matrix_to_tf(parent, child, M)


class FlangeVis(Node):
    def __init__(self, camera_matrix=None, publish_tool=True, tool_calib=None):
        super().__init__('flange_vis')
        self.camera_matrix = matrix_list_to_4x4(
            camera_matrix if camera_matrix else DEFAULT_CAMERA_TO_FLANGE)
        self.publish_tool = publish_tool
        self.tool_calib = list(tool_calib) if tool_calib else None  # 标定结果：TCP 相对法兰 [x,y,z,rx,ry,rz]
        self.sub = self.create_subscription(
            RobotState, '/driver/state', self.on_state, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.marker_pub = self.create_publisher(MarkerArray, '/flange_marker', 10)
        self.get_logger().info(
            'FlangeVis 已启动：发布 flange / camera_link%s'
            % (' / tool_tcp' if publish_tool else ''))

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
        t.child_frame_id = 'flange'
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

        # ── 3) TF：base_link → camera_link（相机 = 法兰 × 相机相对法兰）──
        M_flange = [[R[0][0], R[0][1], R[0][2], x],
                    [R[1][0], R[1][1], R[1][2], y],
                    [R[2][0], R[2][1], R[2][2], z],
                    [0, 0, 0, 1]]
        M_cam = mat_mult4(M_flange, self.camera_matrix)
        cam_tf = matrix_to_tf('base_link', 'camera_link', M_cam)
        cam_tf.header.stamp = now
        self.tf_broadcaster.sendTransform(cam_tf)

        # ── 4) TF：base_link → tool_tcp（当前工具坐标系，可选）──
        if self.publish_tool and len(msg.tool_pose) >= 6:
            tool_tf = pose_to_tf('base_link', 'tool_tcp', msg.tool_pose)
            tool_tf.header.stamp = now
            self.tf_broadcaster.sendTransform(tool_tf)

        # ── 5) TF：flange → tool_calib（六点标定结果：TCP 相对法兰，静态）──
        if self.tool_calib:
            calib_tf = pose_to_tf('flange', 'tool_calib', self.tool_calib)
            calib_tf.header.stamp = now
            self.tf_broadcaster.sendTransform(calib_tf)

        self.get_logger().debug(
            'flange pos=(%.3f, %.3f, %.3f) rpy=(%.3f, %.3f, %.3f) z_axis=(%.3f, %.3f, %.3f)'
            % (x, y, z, rx, ry, rz, z_axis[0], z_axis[1], z_axis[2]))


def main():
    parser = argparse.ArgumentParser(description='法兰 / 相机 / 工具 TF 可视化')
    parser.add_argument('--camera', type=float, nargs=16,
                        default=DEFAULT_CAMERA_TO_FLANGE,
                        metavar='v', help='相机相对法兰矩阵（16 元素行优先）')
    parser.add_argument('--no-tool', action='store_true', help='不发布 tool_tcp 坐标系')
    parser.add_argument('--tool-calib', type=float, nargs=6, default=None,
                        metavar='v',
                        help='六点标定结果（TCP 相对法兰）[x,y,z,rx,ry,rz] m/rad，发布 flange->tool_calib')
    args = parser.parse_args()

    rclpy.init()
    rclpy.spin(FlangeVis(args.camera, not args.no_tool, args.tool_calib))
    rclpy.shutdown()


if __name__ == '__main__':
    main()
