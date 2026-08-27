#!/bin/bash
# ============================================================
#  传感器坐标系 RViz 可视化一键启动
#  ------------------------------------------------------------
#  启动：driver(sim) + robot_state_publisher + flange_vis + RViz2
#  RViz 中 Fixed Frame=base_link，添加 TF 显示即可看到：
#    · flange       法兰坐标系
#    · camera_link  相机坐标系（法兰 × 相机相对法兰）
#    · tool_tcp     当前工具坐标系
#    · 机器人本体模型（robot_state_publisher + URDF）
#
#  用法：
#     bash src/rus_sim_bringup/scripts/run_sensor_tf_rviz.sh
# ============================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BRINGUP_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
PROJ_DIR="$(cd "$BRINGUP_DIR/../.." && pwd)"  # scripts -> rus_sim_bringup -> src -> 项目根

# 确保 ROS2 环境（若未 source 则自动加载）
source "$PROJ_DIR/install/setup.bash" 2>/dev/null || true

URDF="$PROJ_DIR/src/rus_sim_driver/robot_model/fairino3_v6.urdf"
RVIZ_CFG="$PROJ_DIR/src/rus_sim_driver/config/simulation.rviz"

# 生成 robot_description 参数文件（URDF 含引号，用 yaml 块标量安全传递）
PARAM_FILE="$(mktemp /tmp/robot_desc_XXXX.yaml)"
python3 - "$URDF" "$PARAM_FILE" <<'PY'
import sys
desc = open(sys.argv[1]).read()
with open(sys.argv[2], 'w') as f:
    f.write('/**:\n  ros__parameters:\n    robot_description: |\n')
    for line in desc.splitlines():
        f.write('      ' + line + '\n')
PY

PIDS=()

cleanup() {
    echo ""
    echo "停止所有进程..."
    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null || true
    done
    rm -f "$PARAM_FILE"
}
trap cleanup EXIT INT TERM

if [ ! -f "$URDF" ]; then
    echo "错误：找不到 URDF: $URDF"
    exit 1
fi

echo "[1/4] 启动驱动节点 (sim)..."
ros2 run rus_sim_driver rus_sim_driver_node --ros-args -p driver_type:=sim &
PIDS+=($!)

sleep 2
echo "[2/4] 启动 robot_state_publisher..."
ros2 run robot_state_publisher robot_state_publisher --ros-args --params-file "$PARAM_FILE" &
PIDS+=($!)

echo "[3/4] 启动传感器 TF 发布 (flange/camera_link/tool_tcp)..."
python3 "$SCRIPT_DIR/flange_vis.py" &
PIDS+=($!)

sleep 2
echo "[4/4] 启动 RViz2..."
rviz2 -d "$RVIZ_CFG" &
PIDS+=($!)

echo ""
echo "全部启动完成。"
echo "RViz：Fixed Frame=base_link，左侧 Displays 添加 TF 显示，可查看 flange / camera_link / tool_tcp 坐标系。"
echo "按 Ctrl+C 退出。"
wait
