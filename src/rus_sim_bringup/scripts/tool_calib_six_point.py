#!/usr/bin/env python3
# ============================================================
#  六点法工具坐标系标定脚本
#  ------------------------------------------------------------
#  交互式六点法标定（TCP 相对法兰位姿），标定计算由驱动内部
#  （SDK ComputeTool）完成，脚本只负责：
#    1. 依次引导记录 6 个标定点（每对准尖点按一次 Enter）
#    2. 调 compute_tool_calib 计算工具坐标系
#    3. 调 set_tool_coord 写入指定工具坐标系编号并持久化到配置文件
#    4. 可选切换为当前工具坐标系（set_tool_index）
#
#  前置条件：驱动（/driver/command 服务）已启动（本脚本不启动驱动）。
#
#  用法：
#     source install/setup.bash
#     python3 src/rus_sim_bringup/scripts/tool_calib_six_point.py
#     python3 src/rus_sim_bringup/scripts/tool_calib_six_point.py --tool-id 3
#     python3 src/rus_sim_bringup/scripts/tool_calib_six_point.py --skip-record
# ============================================================

import argparse
import math
import threading

import rclpy
from rclpy.node import Node

from rus_sim_interfaces.srv import CommandService
from rus_sim_interfaces.msg import RobotState


class ToolCalibSixPoint(Node):
    def __init__(self):
        super().__init__('tool_calib_six_point')
        self.client = self.create_client(CommandService, '/driver/command')
        self.latest = None
        self.create_subscription(RobotState, '/driver/state', self.on_state, 10)

    def on_state(self, msg):
        self.latest = msg

    def call(self, cmd, args):
        """调用驱动指令服务，返回 Response 或 None"""
        if not self.client.service_is_ready():
            return None
        req = CommandService.Request()
        req.client_id = 0
        req.command = cmd
        req.args = args
        return self.client.call(req)

    def show_current(self):
        """打印当前 TCP 位姿（辅助判断是否对准尖点）"""
        if self.latest and len(self.latest.tool_pose) >= 6:
            t = self.latest.tool_pose
            print('  当前 TCP: (%.3f, %.3f, %.3f) m   姿态 (%.2f, %.2f, %.2f) rad'
                  % (t[0], t[1], t[2], t[3], t[4], t[5]))


def wait_enter(prompt):
    try:
        input(prompt)
        return True
    except (EOFError, KeyboardInterrupt):
        return False


def main():
    parser = argparse.ArgumentParser(description='六点法工具坐标系标定（驱动需已启动）')
    parser.add_argument('--tool-id', type=int, default=1,
                        help='标定结果写入的工具坐标系编号 [0~14]，默认 1')
    parser.add_argument('--skip-record', action='store_true',
                        help='跳过记录步骤直接计算（用于已记录过 6 点的场景）')
    args = parser.parse_args()

    rclpy.init()
    node = ToolCalibSixPoint()
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()

    try:
        print('=' * 60)
        print('六点法工具坐标系标定')
        print('=' * 60)

        # 等待驱动服务
        if not node.client.wait_for_service(timeout_sec=5.0):
            print('错误：/driver/command 服务不可用。')
            print('      请先启动驱动（例如：ros2 run rus_sim_driver rus_sim_driver_node --ros-args -p driver_type:=real）。')
            return
        print('驱动服务就绪 OK')

        # 显示现有工具坐标系配置
        resp = node.call('get_tool_coords', [])
        if resp and resp.success and len(resp.result) >= 2:
            cur_idx, count = int(resp.result[0]), int(resp.result[1])
            print('现有工具坐标系：当前索引=%d，共 %d 个' % (cur_idx, count))
            for i in range(count):
                base = 2 + i * 6
                if base + 6 <= len(resp.result):
                    v = resp.result[base:base + 6]
                    mark = '  <- 当前' if i == cur_idx else ''
                    print('  工具%d: (%.3f, %.3f, %.3f) m / (%.2f, %.2f, %.2f) rad%s'
                          % (i, v[0], v[1], v[2], v[3], v[4], v[5], mark))
        else:
            print('（无法读取工具坐标系配置，将使用默认）')

        # 记录 6 个标定点
        if args.skip_record:
            print('\n[--skip-record] 跳过记录，直接计算（需已记录过 6 个点）')
        else:
            print('\n六点法说明（官方流程）：')
            print('  点 1~4：TCP 精确对准同一标定尖点，依次取 4 个差异较大的姿态（确定 TCP 位置）')
            print('  点 5  ：姿态保持不变，沿工具 X 轴正方向平移一段距离（确定 X 轴方向）')
            print('  点 6  ：姿态保持不变，沿工具 Z 轴正方向平移一段距离（确定 Z 轴方向）\n')
            for n in range(1, 7):
                print('-' * 60)
                node.show_current()
                if n <= 4:
                    tip = 'TCP 对准尖点（姿态 %d/4），按 Enter 记录' % n
                elif n == 5:
                    tip = '姿态不变，沿工具 X 轴正方向平移一段距离后按 Enter'
                else:
                    tip = '姿态不变，沿工具 Z 轴正方向平移一段距离后按 Enter'
                if not wait_enter('[%d/6] %s（Ctrl+C 取消）... ' % (n, tip)):
                    print('\n已取消记录。')
                    return
                resp = node.call('set_tool_calib_point', [float(n)])
                if resp and resp.success:
                    print('  OK 已记录第 %d 点' % n)
                else:
                    msg = resp.message if resp else '服务超时/不可用'
                    print('  X 记录第 %d 点失败：%s' % (n, msg))
                    return
            print('\n6 个点全部记录完成。')

        # 计算工具坐标系
        print('\n正在计算工具坐标系（标定计算在驱动内部完成）...')
        resp = node.call('compute_tool_calib', [])
        if not resp or not resp.success or len(resp.result) < 6:
            msg = resp.message if resp else '服务超时/不可用'
            print('X 计算失败：%s' % msg)
            return
        x, y, z, rx, ry, rz = resp.result[0:6]
        print('-' * 60)
        print('标定结果（TCP 相对法兰）：')
        print('  位置: (%.4f, %.4f, %.4f) m  =  (%.1f, %.1f, %.1f) mm'
              % (x, y, z, x * 1000.0, y * 1000.0, z * 1000.0))
        print('  姿态: (%.4f, %.4f, %.4f) rad =  (%.2f, %.2f, %.2f) deg'
              % (rx, ry, rz, math.degrees(rx), math.degrees(ry), math.degrees(rz)))

        # 写入工具坐标系（持久化）
        print('\n写入工具坐标系（驱动会持久化到配置文件，重启自动恢复）？')
        try:
            inp = input('输入工具坐标系编号 [0~14]（直接回车=默认 %d；q=退出不写入）: '
                        % args.tool_id).strip()
        except (EOFError, KeyboardInterrupt):
            inp = ''
        if inp.lower() == 'q':
            print('已退出，结果未写入。')
            return
        tid = args.tool_id if inp == '' else int(inp)

        resp = node.call('set_tool_coord', [float(tid)] + list(resp.result[0:6]))
        if not resp or not resp.success:
            msg = resp.message if resp else '服务超时/不可用'
            print('X 工具坐标系 %d 写入失败：%s' % (tid, msg))
            return
        print('OK 工具坐标系 %d 写入成功（已持久化）' % tid)

        # 切换为当前工具坐标系
        try:
            inp = input('是否切换为当前工具坐标系（运动参考系随之切换）？[Y/n] ').strip().lower()
        except (EOFError, KeyboardInterrupt):
            inp = 'y'
        if inp != 'n':
            resp = node.call('set_tool_index', [float(tid)])
            if resp and resp.success:
                print('OK 已切换，当前工具索引 = %d' % tid)
            else:
                print('X 切换索引失败')

        print('\n' + '=' * 60)
        print('六点法标定完成')
        print('   工具坐标系 %d 已持久化，驱动重启后自动恢复。' % tid)
        print('=' * 60)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        spin.join(timeout=2.0)


if __name__ == '__main__':
    main()
