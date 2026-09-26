# RUS_Engine 后端开发文档

> RUS-Sim 后端由 **8 个 ROS 2 功能包**组成，对外只有 `rus_sim_bridge` 一个 WebSocket 出口。
> 本页是文档总入口：文档地图 → 环境搭建 → 包总览 → 常用命令 → 话题/服务总览。

## 文档地图

| 文档 | 管什么 | 什么时候看 |
|------|--------|-----------|
| [DevelopmentGuide.md](./DevelopmentGuide.md) | C++ 代码规范（命名 / 目录 / CMake / 注释 / 测试） | 写代码前 |
| [DevelopmentGuide.draft.md](./DevelopmentGuide.draft.md) | **全系统契约初稿**：包清单、启动方式、话题/服务、指令总表、参数表、包内部结构、文档冲突清单 | 查接口事实、评审、写新文档时 |
| [DocExample.md](./DocExample.md) | 单包文档**模板** | 新增包要写文档时 |
| [Protocol/WsProtocol.md](./Protocol/WsProtocol.md) | 前后端 WebSocket 协议（通道、指令、reply/event、state/sensor 线格式） | 前端联调 |
| [Protocol/RecFormat.md](./Protocol/RecFormat.md) | `.rusrec` 记录文件格式（头 / 记录 / 索引 / 回放口径） | 读写录音、体检工具 |
| [Protocol/README.md](./Protocol/README.md) | 协议文档索引与速查表 | 快速定位协议条款 |
| `rus_sim_<包名>/rus_sim_<包名>.md` | 单模块文档：结构 / 输入输出 / 指令 / 参数 / 启动 | 定位到某个模块时 |

> 接口事实**一律以代码为准**；文档与代码不一致时，先改代码或先在此登记（见 draft §11 冲突清单）。

## 后端开发环境搭建

### 系统环境要求
- 操作系统：**Ubuntu 22.04**
- **CMake** 3.18+

### 安装 ROS 2 Humble
- 安装教程见 [ROS 2 Documentation](https://docs.ros.org/en/humble/index.html)

### 安装额外依赖
- **libwebsockets**（`rus_sim_bridge`，系统库 `-lwebsockets`）
- **libzstd**（`rus_sim_perception` 点云压缩，缺失时 CMake 直接报错）
- **librealsense2**（感知直连相机；缺失时自动降级为 stub）
- **Eigen3** 3.4.0、**PCL** 1.21.1、**pcl_conversions** 2.4.5
- **Boost**（graph 组件，`rus_sim_planning`）
- **MuJoCo**（`rus_sim_driver` 仿真后端，自定义 `Findmujoco.cmake`）
- **libigl** 2.5.0（项目内 `third_party/`）
- **Fairino SDK** `libfairino.so.2.1.4`（随包：`src/rus_sim_driver/SDK/`）
- **EAIK**（逆运动学求解，`third_party/EAIK`）

## 项目结构

```
src/
├── rus_sim_interfaces      # 全部消息 / 服务定义（3 msg + 1 srv）
├── rus_sim_utils           # 协议常量、指令定义与校验、路由注册表、位姿工具（header-only）
├── rus_sim_bridge          # 前后端 WebSocket 网关
├── rus_sim_driver          # 机械臂驱动（MuJoCo 仿真 / Fairino 真机）
├── rus_sim_perception      # 感知：相机采集 → 点云预处理 / 建图
├── rus_sim_planning        # 规划：点云表面 → 扫查轨迹 → 伺服执行
├── rus_sim_recorder        # 记录层：录制 / 离线回放 / 体检工具
└── rus_sim_bringup         # 一键启动 + 联调脚本
```

| 包 | 类型 | 一句话职责 | 模块文档 |
|----|------|-----------|---------|
| `rus_sim_interfaces` | ament_cmake | 消息 / 服务定义 | [文档](./rus_sim_interfaces/rus_sim_interfaces.md) |
| `rus_sim_utils` | header-only（INTERFACE） | 协议与指令的**唯一定义处** | [文档](./rus_sim_utils/rus_sim_utils.md) |
| `rus_sim_bridge` | ament_cmake | WS 网关（纯转发，无业务逻辑） | [文档](./rus_sim_bridge/rus_sim_bridge.md) |
| `rus_sim_driver` | ament_cmake | 机械臂驱动 + 状态发布（125 Hz） | [文档](./rus_sim_driver/rus_sim_driver.md) |
| `rus_sim_perception` | ament_cmake | 相机 → 点云（预处理 / 建图 / 压缩帧） | [文档](./rus_sim_perception/rus_sim_perception.md) |
| `rus_sim_planning` | ament_cmake | 轨迹生成 + 插值 + 伺服执行 | [文档](./rus_sim_planning/rus_sim_planning.md) |
| `rus_sim_recorder` | ament_cmake | 录制 `.rusrec` + 离线回放 | [文档](./rus_sim_recorder/rus_sim_recorder.md) |
| `rus_sim_bringup` | ament_cmake（仅 launch） | 一键拉起全系统 | [文档](./rus_sim_bringup/rus_sim_bringup.md) |

## 快速开始

```bash
source /opt/ros/humble/setup.bash
cd RUS_Engine
colcon build
source install/setup.bash
```

```bash
# 一键全栈
ros2 launch rus_sim_bringup rus_sim.launch.py
ros2 launch rus_sim_bringup rus_sim.launch.py record:=true          # 连录制一起拉起
ros2 launch rus_sim_bringup rus_sim.launch.py record_autostart:=false  # 录制备好待命，等前端 recorder_start

# 分开启动（调试单模块）
ros2 launch rus_sim_bridge      bridge.launch.py        # WS 网关
ros2 launch rus_sim_driver      driver.launch.py        # 驱动 + robot_state_publisher
ros2 launch rus_sim_planning    planning.launch.py      # 规划
ros2 launch rus_sim_perception  perception.launch.py    # 感知
ros2 launch rus_sim_recorder    recorder.launch.py      # 录制（默认录到 records/）
ros2 launch rus_sim_recorder    replayer.launch.py      # 离线回放（默认载入 records/ 第 0 个）
ros2 launch rus_sim_driver      keyboard_control.launch.py  # 键盘点动

# 录制文件体检（离线，不依赖 ROS 图）
ros2 run rus_sim_recorder rus_sim_recorder_inspect records/run_*.rusrec --check-crc
```

## 话题 / 服务总览

| 名称 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/driver/state` | `RobotState` | driver → bridge / planning / perception / recorder | 关节 + 法兰 + TCP 状态，125 Hz |
| `/joint_states` | `sensor_msgs/JointState` | driver → robot_state_publisher | 关节名 `j1..j6` |
| `/camera/camera/depth/color/points` | `PointCloud2` | 相机 → perception | 仅 `source=ros_topic` 时使用 |
| `/preprocessed_cloud` | `PointCloud2` | perception → planning | 地图快照或当前帧（规划输入） |
| `/perception/frame` | `PointCloud2` | perception → RViz | 当前处理帧 |
| `/sensor/pointcloud` | `SensorFrame` | perception → bridge / recorder | zstd + int16 量化压缩帧 |
| `/planned_trajectory` | `geometry_msgs/PoseArray` | planning → RViz | 规划轨迹调试可视化 |
| `/module_events` | `ModuleEvent` | planning / replayer → bridge | 子模块事件上报 |
| `/driver/command` | `CommandService` | bridge / planning → driver | 驱动指令 |
| `/planning/command` | `CommandService` | bridge → planning | 规划指令 |
| `/perception/command` | `CommandService` | bridge → perception | 感知指令 |
| `/recorder/command` | `CommandService` | bridge → recorder | 录制开关（3 条 `recorder_*`） |
| `/replayer/command` | `CommandService` | bridge → replayer | 离线回放（10 条 `replay_*`） |
| WS `/control` `/state` `/sensor` | WebSocket | 前端 ⇄ bridge | 见 [WsProtocol.md](./Protocol/WsProtocol.md) |

## 文档维护约定

1. **每个包一份文档**：`docs/<包名>/<包名>.md`，格式参考 [DocExample.md](./DocExample.md)（结构 / 输入输出 / 指令 / 参数 / 启动）。
2. **接口变更三处同步**：代码 → `docs/Protocol/*` → 对应模块文档；跨模块契约同时更新 `DevelopmentGuide.draft.md` 相应章节。
3. **状态标记**：功能清单用 ✅ 已完成 / 🚧 进行中 / 📝 待办 / ❌ 废弃，让人一眼看出实现程度。
4. **相对链接**：文档内引用其它文档一律用仓库内相对路径，便于整仓迁移。
