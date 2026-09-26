# RUS-Sim 超声机器人仿真平台

## 项目简介

RUS-Sim（Robotic Ultrasound Simulator）是一个面向超声扫查机械臂的数字孪生仿真平台，旨在为自动化扫查算法和 AI 辅助诊断模块的研发提供安全、可控的集成验证环境。

## 核心功能

- **3D 交互仿真**：构建患者与机械臂的数字孪生环境，支持可视化交互
- **真实超声设备的图像接入**：实时采集与显示真实超声设备的视频流
- **机械臂运动控制**：支持正逆运动学解算、轨迹规划与恒力控制
- **数据可视化与记录**：实时监控机械臂状态、接触力、图像数据
- **自主路径规划**：自动生成扫查路径，实现标准切面的自主定位与采集
- **智能化 AI 模型的接入**：支持图像分割、病灶检测等深度学习模型

## 系统组成

前端（Avalonia）通过 WebSocket 接入；后端是 8 个 ROS 2 功能包，前后端之间只有
`rus_sim_bridge` 一个出口，模块之间用 ROS 2 服务 / 话题解耦。

```text
       前端（Avalonia）
         │  WS /control  ⇄  指令 / 回执 / 事件
         │  WS /state   ←   机械臂状态流（125 Hz，可丢帧）
         │  WS /sensor  ←   压缩点云帧（可丢帧）
         ▼
   rus_sim_bridge            # 唯一网关：指令路由 + 话题转发
         │ CommandService
         ├──→ rus_sim_driver      机械臂驱动（MuJoCo 仿真 / Fairino 真机）
         ├──→ rus_sim_planning    点云 → 扫查轨迹 → 125 Hz 伺服下发
         ├──→ rus_sim_perception  相机 → 点云预处理 / 建图 / 压缩帧
         ├──→ rus_sim_recorder    录制开关（recorder_*）
         └──→ rus_sim_replayer    离线回放（replay_*）

   模块间数据（ROS 2 话题）：
     /driver/state         driver → bridge / planning / perception / recorder
     /preprocessed_cloud   perception → planning（规划输入）
     /sensor/pointcloud    perception → bridge / recorder（落盘）
     /module_events        planning / replayer → bridge
```

| 包 | 一句话职责 | 文档 |
|----|-----------|------|
| `rus_sim_bridge` | 前后端 WebSocket 网关（指令路由 + 状态/感知流转发） | [docs/rus_sim_bridge](docs/rus_sim_bridge/rus_sim_bridge.md) |
| `rus_sim_driver` | 机械臂驱动：MuJoCo 仿真 / Fairino 真机、运动学、伺服 | [docs/rus_sim_driver](docs/rus_sim_driver/rus_sim_driver.md) |
| `rus_sim_perception` | 相机采集 → 点云预处理 / 建图 → 状态对齐压缩帧 | [docs/rus_sim_perception](docs/rus_sim_perception/rus_sim_perception.md) |
| `rus_sim_planning` | 点云表面 → 扫查轨迹生成 / 插值 → 伺服执行 | [docs/rus_sim_planning](docs/rus_sim_planning/rus_sim_planning.md) |
| `rus_sim_recorder` | 记录层：双通道落盘 `.rusrec` + 离线回放 + 体检工具 | [docs/rus_sim_recorder](docs/rus_sim_recorder/rus_sim_recorder.md) |
| `rus_sim_interfaces` | 全部消息 / 服务定义（3 msg + 1 srv） | [docs/rus_sim_interfaces](docs/rus_sim_interfaces/rus_sim_interfaces.md) |
| `rus_sim_utils` | 协议常量、指令定义与校验、路由注册表、位姿工具（header-only） | [docs/rus_sim_utils](docs/rus_sim_utils/rus_sim_utils.md) |
| `rus_sim_bringup` | 一键启动全栈 + 联调脚本 | [docs/rus_sim_bringup](docs/rus_sim_bringup/rus_sim_bringup.md) |

## 快速开始

> 环境：**Ubuntu 22.04 + ROS 2 Humble**（依赖清单见 [docs/README.md](docs/README.md)）。

```bash
source /opt/ros/humble/setup.bash

# 一次性：安装 ROS 侧依赖
cd RUS_Engine
rosdep install --from-paths src --ignore-src -r -y

# 编译
colcon build
source install/setup.bash

# 一键启动（驱动 + 规划 + 感知 + 桥接）
ros2 launch rus_sim_bringup rus_sim.launch.py

# 需要录制时（默认启动即录，写到 records/）
ros2 launch rus_sim_bringup rus_sim.launch.py record:=true
```

常用附加能力：

```bash
# 录制文件体检（离线，不依赖 ROS 图）
ros2 run rus_sim_recorder rus_sim_recorder_inspect records/run_*.rusrec --check-crc

# 离线回放（把录音按时间轴重发回原话题，前端可视化复盘）
ros2 launch rus_sim_recorder replayer.launch.py

# 感知流通路联调（ROS 侧 + WS 侧探针）
python3 tools/ws_probe/probe_sensor.py
python3 tools/ws_probe/ws_sensor_check.py
```

## 目录结构

```
RUS_Engine/
├── src/                  # ROS 2 功能包（8 个，全部以 rus_sim_ 开头）
│   ├── rus_sim_interfaces/   # 消息 / 服务定义
│   ├── rus_sim_utils/        # 协议与指令定义（header-only）
│   ├── rus_sim_bridge/       # WS 网关
│   ├── rus_sim_driver/       # 机械臂驱动
│   ├── rus_sim_perception/   # 感知（相机 → 点云）
│   ├── rus_sim_planning/     # 规划
│   ├── rus_sim_recorder/     # 录制 / 回放
│   └── rus_sim_bringup/      # 启动文件与联调脚本
├── docs/                 # 文档（入口：docs/README.md）
├── tools/ws_probe/       # 感知流联调探针
├── records/              # 录制产物 .rusrec（运行时生成）
└── build/ install/ log/  # colcon 产物（不入库）
```

## 文档链接

| 文档 | 内容 |
|------|------|
| [docs/README.md](docs/README.md) | **后端文档入口**：环境搭建、包总览、常用命令 |
| [docs/DevelopmentGuide.md](docs/DevelopmentGuide.md) | C++ 代码规范（命名 / 目录 / CMake / 注释） |
| [docs/Protocol/WsProtocol.md](docs/Protocol/WsProtocol.md) | 前后端 WebSocket 协议（通道、指令、事件） |
| [docs/Protocol/RecFormat.md](docs/Protocol/RecFormat.md) | `.rusrec` 记录文件格式 |
| [docs/Protocol/README.md](docs/Protocol/README.md) | 协议文档索引与速查 |
| [docs/rus_sim_*/](docs/README.md) | 各模块文档（结构 / 话题 / 指令 / 参数 / 启动） |

> 前端（Avalonia）工程独立维护，不在本仓库内；前端对接以后端协议文档为准。