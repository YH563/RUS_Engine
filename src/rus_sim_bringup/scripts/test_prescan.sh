#!/bin/bash
# ════════════════════════════════════════════════════════════════════
#  预扫查流程控制脚本（系统已由用户自行启动）
#  ────────────────────────────────────────────────────────────────────
#  前提：整系统已运行（感知已加载并发布点云）。
#  流程：
#    1. pre_scan_done → planning 取最新点云初始化轨迹生成器
#    2. set_start_pose / set_end_pose（读取起终点文件）
#    3. plan
#    4. execute（sim_driver 仿真运动）
#
#  用法：
#    bash src/rus_sim_bringup/scripts/test_prescan.sh [起终点文件]
#    （默认 /tmp/test_cloud_poses.txt；由 rus_sim_gen_test_cloud 生成）
# ════════════════════════════════════════════════════════════════════
set -euo pipefail

WORKSPACE="$(cd "$(dirname "$0")/../../.." && pwd)"
POSES_PATH="${1:-/tmp/test_cloud_poses.txt}"

# ROS setup.bash 内部引用未定义变量，source 前临时关闭 -e -u
set +eu
source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"
set -eu

if [ ! -f "$POSES_PATH" ]; then
    echo "起终点文件不存在: $POSES_PATH"
    echo "请先运行: ros2 run rus_sim_perception rus_sim_gen_test_cloud [点云路径] $POSES_PATH"
    exit 1
fi

START_XYZ=$(sed -n '1p' "$POSES_PATH")
GOAL_XYZ=$(sed -n '2p' "$POSES_PATH")
START_ARGS="${START_XYZ// /,}"
GOAL_ARGS="${GOAL_XYZ// /,}"

echo "════════════════════════════════════════════"
echo "  预扫查流程控制（系统已在运行）"
echo "  起点: $START_XYZ"
echo "  终点: $GOAL_XYZ"
echo "════════════════════════════════════════════"

# ── 1. pre_scan_done：planning 取最新点云初始化轨迹生成器 ────────
echo "[1/4] 发送 pre_scan_done..."
until ros2 service list 2>/dev/null | grep -q /planning/command; do sleep 1; done
ros2 service call /planning/command rus_sim_interfaces/srv/CommandService \
  "{client_id: 0, command: 'pre_scan_done', args: []}"

# ── 2. 设置起终点 ───────────────────────────────────────────────
echo "[2/4] 设置起终点..."
ros2 service call /planning/command rus_sim_interfaces/srv/CommandService \
  "{client_id: 0, command: 'set_start_pose', args: [$START_ARGS]}"
ros2 service call /planning/command rus_sim_interfaces/srv/CommandService \
  "{client_id: 0, command: 'set_end_pose', args: [$GOAL_ARGS]}"

# ── 3. 规划 ─────────────────────────────────────────────────────
echo "[3/4] plan..."
ros2 service call /planning/command rus_sim_interfaces/srv/CommandService \
  "{client_id: 0, command: 'plan', args: []}"

# ── 4. 执行（sim_driver 仿真运动）───────────────────────────────
echo "[4/4] execute（在 RViz 观察机械臂沿轨迹运动）..."
ros2 service call /planning/command rus_sim_interfaces/srv/CommandService \
  "{client_id: 0, command: 'execute', args: []}"

echo "════════════════════════════════════════════"
echo "  运动执行中，在 RViz 观察。"
echo "════════════════════════════════════════════"
