#!/usr/bin/env python3
# ============================================================
#  法兰 TF + 标定工具坐标系 TF 可视化（RViz）
#  ------------------------------------------------------------
#  订阅 /driver/state，发布：
#    · base_link → flange      法兰坐标系（flange_pos，XYZABC m/rad）
#    · flange → tool_calib     六点标定的工具坐标系（TCP 相对法兰，参数传入）
#
#  前置：驱动已启动（/driver/state 有数据）；如要看机器人模型还需
#        robot_state_publisher（提供 base_link 等模型 TF）。
#
#  用法：
#     source install/setup.bash
#     python3 src/rus_sim_bringup/scripts/tool_tf_vis.py \
#       --tool-calib -0.0127 -0.0117 0.2204 0.0432 0.0608 -2.3443
#   RViz：Fixed Frame=base_link，Add → TF，勾选 flange / tool_calib。
# ============================================================

import argparse
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

from rus_sim_interfaces.msg import RobotState


# ── 位姿 / 旋转工具（固定轴 XYZ：R = Rz·Ry·Rx，与驱动 flange_pos 一致）──
def mat_mult3(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)]
            for i in range(3)]


def rpy_to_rotation(rx, ry, rz):
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    Rx = [[1, 0, 0], [0, cx, -sx], [0, sx, cx]]
    Ry = [[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]]
    Rz = [[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]]
    return mat_mult3(Rz, mat_mult3(Ry, Rx))


def quat_from_matrix(R):
    """3x3 旋转矩阵 → (x, y, z, w)"""
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


def pose_to_tf(parent, child, pose6):
    """[x,y,z,rx,ry,rz] → TransformStamped"""
    x, y, z, rx, ry, rz = pose6[0], pose6[1], pose6[2], pose6[3], pose6[4], pose6[5]
    R = rpy_to_rotation(rx, ry, rz)
    t = TransformStamped()
    t.header.frame_id = parent
    t.child_frame_id = child
    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.translation.z = z
    q = quat_from_matrix(R)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]
    return t


class ToolTfVis(Node):
    def __init__(self, tool_calib):
        super().__init__('tool_tf_vis')
        self.tool_calib = list(tool_calib) if tool_calib else None
        self.tf_broadcaster = TransformBroadcaster(self)
        self.sub = self.create_subscription(
            RobotState, '/driver/state', self.on_state, 10)
        self.get_logger().info(
            'ToolTfVis 已启动：发布 flange' + (' 与 tool_calib(标定结果)' if self.tool_calib else ''))
        if self.tool_calib:
            self.get_logger().info(
                'tool_calib 相对法兰: 位置(%.3f, %.3f, %.3f) m  姿态(%.3f, %.3f, %.3f) rad'
                % tuple(self.tool_calib))

    def on_state(self, msg):
        if len(msg.flange_pos) < 6:
            return
        now = self.get_clock().now().to_msg()

        # 法兰坐标系（base_link -> flange）
        flange = pose_to_tf('base_link', 'flange', msg.flange_pos)
        flange.header.stamp = now
        self.tf_broadcaster.sendTransform(flange)

        # 标定工具坐标系（flange -> tool_calib，相对法兰）
        if self.tool_calib:
            calib = pose_to_tf('flange', 'tool_calib', self.tool_calib)
            calib.header.stamp = now
            self.tf_broadcaster.sendTransform(calib)

        self.get_logger().debug(
            'flange=(%.3f, %.3f, %.3f) rpy=(%.2f, %.2f, %.2f)'
            % (msg.flange_pos[0], msg.flange_pos[1], msg.flange_pos[2],
               msg.flange_pos[3], msg.flange_pos[4], msg.flange_pos[5]))


def main():
    parser = argparse.ArgumentParser(description='法兰 + 标定工具坐标系 TF 可视化')
    parser.add_argument('--tool-calib', type=float, nargs=6, default=None,
                        metavar='v',
                        help='六点标定结果（TCP 相对法兰）[x,y,z,rx,ry,rz] m/rad，发布 flange->tool_calib')
    args = parser.parse_args()

    rclpy.init()
    rclpy.spin(ToolTfVis(args.tool_calib))
    rclpy.shutdown()


if __name__ == '__main__':
    main()
