# RUS_Engine 后端规范（初稿 v0.1）

> **状态：初步草稿，仅供评审。**
> 最终目标是让 `docs/DevelopmentGuide.md` 成为唯一权威规范；本稿先以「一个文件写清全部后端契约」的形式给出，评审通过后再合并/替换。
>
> **本稿所有接口事实以当前代码为准**，正文中的代码位置即为验证依据。
> 与现有文档冲突的地方集中列在 **§11**，不在此处逐条摭掩。
>
> **核验基准（重要）**：初稿核对于 commit `605a59e`（`main`）**加上工作区未提交改动**，因此以下两条按「改后」状态记录，
> 合并正式文档时请确认对应代码已入库：
> - `command_types.hpp::MoveL::ParseArgs` 放宽为 `size() >= 3`（见 §6.3）；
> - `config/driver_params.yaml::driver_type` 被本地改为 `"real"`（见 §8.4）。

---

## 1. 系统概览

### 1.1 功能包（实际存在的 8 个）

| 包 | 类型 | 一句话职责 | 主要产物 |
|----|------|-----------|---------|
| `rus_sim_interfaces` | ament_cmake（接口） | 全部消息/服务定义 | 3 msg + 1 srv |
| `rus_sim_utils` | **header-only**（INTERFACE） | 协议常量、指令定义、路由注册表、位姿工具 | 无编译单元（`src/rus_sim_utils/CMakeLists.txt`） |
| `rus_sim_bridge` | ament_cmake | 前后端 WebSocket 网关（纯转发，无业务逻辑） | 库 `bridge_core`、可执行 `rus_sim_bridge_node` |
| `rus_sim_driver` | ament_cmake | Fairino 机器人驱动（真实 + MuJoCo 仿真） | 库 `driver_core`、可执行 `rus_sim_driver_node` |
| `rus_sim_perception` | ament_cmake | 点云实时建图 / 预处理 → `/preprocessed_cloud` | 库 `perception_core`、`rus_sim_perception_node`、工具 `rus_sim_gen_test_cloud` |
| `rus_sim_planning` | ament_cmake | 点云轨迹生成 + 插值 + 伺服执行 | 库 `planning_core`、可执行 `rus_sim_planning_node` |
| `rus_sim_recorder` | ament_cmake | 记录层：双路数据流落盘 `.rusrec`（离线复盘） | 库 `rec_storage`（纯 std）/ `recorder_core`、可执行 `rus_sim_recorder_node`、工具 `rus_sim_recorder_inspect` |
| `rus_sim_bringup` | ament_cmake（仅 launch） | 一键拉起全系统 | 无编译单元，只 install `launch/` |

### 1.2 节点 / 可执行 / 端口

| 节点名 | 可执行文件 | 关键对外接口 |
|--------|-----------|-------------|
| `bridge_node` | `ros2 run rus_sim_bridge rus_sim_bridge_node` | ws://0.0.0.0:8765（路径 `/control` `/state` `/sensor`） |
| `driver_node` | `ros2 run rus_sim_driver rus_sim_driver_node` | 服务 `/driver/command`；话题 `/driver/state`、`/joint_states` |
| `planning_node` | `ros2 run rus_sim_planning rus_sim_planning_node` | 服务 `/planning/command`；话题 `/planned_trajectory` |
| `perception_node` | `ros2 run rus_sim_perception rus_sim_perception_node` | 服务 `/perception/command`；话题 `/preprocessed_cloud`、`/perception/frame`、`/sensor/pointcloud` |
| `recorder_node` | `ros2 run rus_sim_recorder rus_sim_recorder_node` | 只订阅（`/driver/state`、`/sensor/pointcloud`）；落地文件 `records/*.rusrec` |
| （工具） | `ros2 run rus_sim_recorder rus_sim_recorder_inspect` | 离线体检 `.rusrec`（无需 ROS 图，可直跑 `install/.../lib/rus_sim_recorder/` 下的可执行） |

> 可执行名一律 `<包名>_node`，与包内 `project()` 名相同但**不是** `project()` 名的自动结果，均为显式 `add_executable()`（见各包 CMakeLists）。

### 1.3 第三方依赖（按包，取自 CMakeLists / package.xml）

| 依赖 | 使用方 | 说明 |
|------|--------|------|
| libwebsockets | bridge | 系统库，`find_library(WEBSOCKETS_LIB websockets REQUIRED)` |
| libzstd | perception | 点云压缩编码，缺失则 CMake `FATAL_ERROR` |
| librealsense2 | perception | RealSense 直连数据源（`source=realsense`）；`find_package(realsense2 QUIET)`，缺失自动降级为 stub（`-DRUS_SIM_WITH_REALSENSE=OFF` 可显式关闭） |
| PCL / pcl_conversions | perception, planning | 点云算法 |
| Eigen3 | utils(driver 无), driver, planning, perception | 数学库 |
| Boost（graph 组件） | planning | `find_package(Boost REQUIRED COMPONENTS graph)` |
| MuJoCo | driver | 自定义 `Findmujoco.cmake`（`src/rus_sim_driver/cmake/`） |
| Fairino SDK (`libfairino.so.2.1.4`) | driver | 随包安装：`src/rus_sim_driver/SDK/libfairino` |
| EAIK（IK 求解） | driver | `third_party/EAIK`，导出 `EAIK` / `EAIK_Kinematics` / `SUBPROBLEMS` |
| urdf / ament_index_cpp | driver | 读 URDF、定位包路径 |

> **未使用**：MoveIt、pinocchio、libigl、shape_msgs（旧文档中列出的依赖已不在当前代码里）。

---

## 2. 分层架构与数据流

```
                        ┌──────────────────────────────┐
   前端 (Avalonia)  ⇄ WS │  rus_sim_bridge (纯网关)      │
   /control /state /sensor│  ├─ WsServer（多通道）        │
                        │  └─ CommandDispatcher        │
                        │       ├ 解析：Cmd::ParseCommand
                        │       ├ 路由：CommandRegistry
                        │       └ 扇出：ROS2 Service 调用
                        └───┬──────────┬──────────┬────┘
                            │ /planning/command │ /perception/command
                            ▼          ▼          ▼
                  ┌──────────────┐ ┌──────────┐ ┌───────────────┐
                  │ planning_node│ │perception│ │  driver_node  │
                  │ 轨迹生成+插值 │ │ 建图+滤波 │ │ sim / real    │
                  └──────┬───────┘ └────┬─────┘ └──────┬────────┘
                         │ servo_cart   │ /preprocessed_cloud
                         └─────────────►│◄─────────────┘
                                        │
              /driver/state (RobotState, 125Hz) → bridge / planning / perception
              /module_events (ModuleEvent)      → bridge（转为前端 event）
```

要点：

- **bridge 是纯网关**：不做业务判断，只解析 / 路由 / 扇出 / 回执聚合（`bridge_node.hpp` 类注释）。
- **子模块之间直连**，不经过 bridge：planning → driver 走 `/driver/command`；perception → planning 走 `/preprocessed_cloud` 话题。
- **记录层旁路**：`rus_sim_recorder` 只订阅（`/driver/state` + `/sensor/pointcloud`）不发布，
  与其他模块之间没有控制流依赖 —— 录制开关不影响任何在线链路（掉盘/写失败只会丢自己的数据）。
- **线程模型**（两端都要记住）：
  - bridge：WS 事件循环线程收 command → 入队；ROS 执行器线程按 `drain_ms`（默认 10ms）出队分发；另有 200ms 超时检测定时器。
  - planning / perception / driver：`MultiThreadedExecutor`（各自 `src/main.cpp`），perception 的处理定时器单独放在 `MutuallyExclusive` 回调组内。

---

## 3. 构建与运行

### 3.1 环境

- Ubuntu 22.04 + ROS 2 Humble；CMake ≥ 3.18；C++17（各包 `CMAKE_CXX_STANDARD 17`）。
- 系统包：`libwebsockets-dev`、`libzstd-dev`、`libpcl-dev`、`libeigen3-dev`、`libboost-graph-dev`、`liburdf-dev`。
- 供应商库：Fairino SDK 放在 `src/rus_sim_driver/SDK/libfairino/{include,lib}`；MuJoCo 通过 `MUJOCO_DIR` 或 `Findmujoco.cmake` 定位。

### 3.2 编译

```bash
# 全量
cd /home/hp/RUS_Sim/RUS_Engine && colcon build --symlink-install

# 单包（含依赖）
colcon build --symlink-install --packages-up-to rus_sim_planning

# 只跑某个包
source install/setup.bash && ros2 run rus_sim_perception rus_sim_perception_node \
  --ros-args --params-file src/rus_sim_perception/config/perception_params.yaml
```

`driver_core` / `bridge_core` / `planning_core` / `perception_core` 均安装到 `lib/`；可执行文件安装到 `lib/<包名>/`（driver 另装 `libfairino.so*` 软链与 `scripts/keyboard_control.py`）。

### 3.3 启动

```bash
# 一键全系统（bridge + driver + planning + perception；record:=true 时含录制）
ros2 launch rus_sim_bringup rus_sim.launch.py
ros2 launch rus_sim_bringup rus_sim.launch.py record:=true

# 分开启动
ros2 launch rus_sim_bridge bridge.launch.py        # WS 网关
ros2 launch rus_sim_driver driver.launch.py        # 驱动 + robot_state_publisher + RViz
ros2 launch rus_sim_planning planning.launch.py    # 规划
ros2 launch rus_sim_perception perception.launch.py# 感知
ros2 launch rus_sim_recorder recorder.launch.py    # 录制（默认录到 records/）
ros2 launch rus_sim_driver keyboard_control.launch.py  # 键盘点动

# 录制文件体检（离线，不依赖 ROS 图）
ros2 run rus_sim_recorder rus_sim_recorder_inspect records/run_*.rusrec --check-crc
```

`rus_sim.launch.py` 只是用 `IncludeLaunchDescription` 组合上面四个 launch（`src/rus_sim_bringup/launch/rus_sim.launch.py`）；
录制默认**不打开**：`record:=true` 才把 `rus_sim_recorder/launch/recorder.launch.py` 一并拉起。
记录文件格式见 `docs/Protocol/RecFormat.md`。

---

## 4. 通信契约

### 4.1 服务（`rus_sim_interfaces/srv/CommandService`）

三处完全同构，只是服务名不同：

| 服务名 | 提供方 | 调用方 |
|--------|--------|--------|
| `/driver/command` | driver_node | bridge（路由）、planning（servo/movel/stop 直发） |
| `/planning/command` | planning_node | bridge（路由） |
| `/perception/command` | perception_node | bridge（路由） |

```text
# 请求
uint32   client_id   # 前端 command id（bridge 透传；0 = 无），用于事件回关联 ack_id
string   command
float64[] args
---
# 响应
bool     success
string   message
float64[] result
```

### 4.2 话题

| 话题 | 类型 | 方向 | 频率 / QoS | 说明 |
|------|------|------|-----------|------|
| `/driver/state` | `rus_sim_interfaces/msg/RobotState` | driver → bridge / planning / perception / recorder | 125 Hz（`create_wall_timer(8ms)`），`frame_id=base_link` | 关节 + 法兰 + TCP 状态 |
| `/joint_states` | `sensor_msgs/msg/JointState` | driver → robot_state_publisher / RViz | 125 Hz | 关节名 `j1..j6` |
| `/camera/camera/depth/color/points` | `sensor_msgs/msg/PointCloud2` | realsense2_camera → perception | 受相机驱动 | 输入点云（**仅 `source=ros_topic` 时使用**；`source=realsense` 直连不经过此话题） |
| `/preprocessed_cloud` | `PointCloud2` | perception → planning | `rolling` / `accumulate`：默认 0.5 Hz 地图快照全量（`map_publish_period: 2.0`）；`none`：处理周期 10 Hz 当前帧。QoS `transient_local` | 地图快照或当前帧（base_link），**planning 规划输入** |
| `/perception/frame` | `PointCloud2` | perception → RViz | 处理周期 10 Hz，QoS `transient_local` | 当前帧（实时可视化） |
| `/sensor/pointcloud` | `rus_sim_interfaces/msg/SensorFrame` | perception → bridge、recorder | 与 `/preprocessed_cloud` 同频，QoS `transient_local` | 压缩帧（zstd + int16 量化 + `range_min/max` + `scope`）；bridge 转发为 WS `/sensor` 二进制帧 |
| `/module_events` | `rus_sim_interfaces/msg/ModuleEvent` | planning → bridge | 事件触发 | 子模块事件上报 |
| `/planned_trajectory` | `geometry_msgs/msg/PoseArray` | planning → RViz | 每次 `plan` 一次 | 规划轨迹调试可视化 |

> 点云来源由 `perception_node` 的 `source` 参数选定（`config/perception_params.yaml`，实现在 `src/rus_sim_perception/src/camera/`）：
> `realsense`（librealsense2 直连，默认）/ `ros_topic`（订阅 `/camera/...`）/ `replay`（离线 PCD 循环回放，无设备联调）/ `auto`（有设备走直连，否则回落 `ros_topic`）。
> ⚠️ 直连与 `realsense2_camera` **互斥**：同一台相机不能被两个进程同时打开（`Device or resource busy`）。

> bridge 订阅三个话题（`bridge_node.cpp`）：`/driver/state`（→ `/state`）、`/module_events`
> （→ `/control` 的 event）、`/sensor/pointcloud`（→ `/sensor` 二进制帧；话题由 `sensor_topic`
> 参数指定，`forward_sensor: false` 可关闭）。感知订阅的 QoS 必须与发布端匹配
> （reliable + `transient_local`），否则 DDS 不建通路（一帧都收不到）。

> 📼 `rus_sim_recorder` 是当前 `/sensor/pointcloud` 的**唯一订阅者**（录制落盘用，见 §9.7 / `docs/Protocol/RecFormat.md`）。

### 4.3 消息定义（字段级，与 `.msg` 一一对应）

**`RobotState.msg`**（驱动发布，125Hz）

```text
std_msgs/Header header      # 时间戳（仿真/驱动时间）
float64[] joint_pos         # [rad]
float64[] joint_vel         # [rad/s]
float64[] joint_acc         # [rad/s²]
float64[] effort            # [Nm]
float64[] flange_pos        # [x,y,z,rx,ry,rz] 法兰位姿，m + rad
int32     tool_index        # 当前工具坐标系索引 0~14，0 = 法兰
float64[] tool_pose         # 当前 TCP 位姿（基座系，XYZABC，m/rad）
```

**`ModuleEvent.msg`**（子模块 → bridge）

```text
string    event             # pre_scan_done / plan_done / scan_done / motion_done / error
bool      success
string    message
float64[] result
uint32    client_id         # bridge 透传的前端 command id → 前端 event.ack_id
string    module            # "driver" / "planning" / "perception"
builtin_interfaces/Time stamp
```

**`SensorFrame.msg`**（感知 → bridge / recorder）

```text
uint8  TYPE_POINTCLOUD = 0
uint8  TYPE_IMAGE      = 1
uint8  TYPE_ULTRASOUND = 2
uint8   type
string  encoding      # payload 压缩算法："zstd" / "raw" / ...
builtin_interfaces/Time stamp
uint32  seq           # 每类型独立递增
string  frame_id      # 点云坐标系（base_link）
string  scope         # 数据语义："frame"（当前帧）/ "map"（地图快照）见 command_defs.hpp::SensorScope
# pointcloud 元数据
uint32   points
string[] fields
string   dtype
float64[] range_min   # int16 量化包围盒（前端反量化必需）
float64[] range_max
# image 元数据（预留）
uint32  width, height, step
string  image_encoding # 压缩前像素格式（编码算法在 encoding）
uint8[] data          # 压缩后 payload
```

**`CommandService.srv`**：见 §4.1。

---

## 5. WebSocket 协议（bridge 网关）

> 详细线格式见 `docs/Protocol/WsProtocol.md`；本节只写与代码强绑定的部分，路径/指令名/事件名的**唯一定义处是 `rus_sim_utils/command_defs.hpp`**。

### 5.1 通道（同端口，按路径）

| 路径 | 常量 | 内容 | 可靠性 |
|------|------|------|--------|
| `/control` | `WsPath::kControl` | `command` / `reply` / `event` | 可靠（按 session id 入队推送） |
| `/state` | `WsPath::kState` | `state` 高频流 | 覆盖式（只保留最新一帧） |
| `/sensor` | `WsPath::kSensor` | 二进制感知帧 | 覆盖式（只推最新一帧，慢客户端丢帧） |

端口参数 `ws_port`（默认 8765）；**端口被占用会自动尝试 +1 / +2**（`ws_server.cpp` 最多 3 次）。

### 5.2 通路 A：command

```json
{ "id": 1, "cmd": "is_motion_done", "args": [] }
```

- 只读 `id` / `cmd` / `args`；一条完整 JSON 以 `}` 收尾即触发处理（`ws_server.cpp` `LWS_CALLBACK_RECEIVE`）。
- JSON 解析失败 → `reply(id=0, success=false, message="malformed command")`。
- 参数不合法 → `invalid args for command: <cmd>`；未注册 → `unknown command: <cmd>`（`command_dispatcher.cpp::Dispatch`）。
- 超时 → `reply(success=false, message="timeout")`；下游服务未就绪 → `service unavailable: <服务名>`。

### 5.3 通路 B：reply / event（同构）

```json
{ "type": "reply", "id": 1, "success": true,  "message": "ok", "result": [1.0] }
{ "type": "event", "id": 0, "ack_id": 3, "event": "plan_done", "success": true, "message": "", "result": [] }
```

- `reply` 只回「发起该指令的那条连接」；`event` 广播给所有 `/control` 连接。
- 扇出指令：并发下发所有目标，**全部成功才 success**，`result` 按目标顺序拼接，`message` 取首个失败信息（`finish_fanout`）。

### 5.4 state

```json
{ "type": "state", "timestamp": 1234.5, "frame_rate": 125.0,
  "joint_pos": [...], "joint_vel": [...], "joint_acc": [...], "effort": [...],
  "flange_pos": [...], "tool_index": 0, "tool_pose": [...] }
```

`frame_rate` 由 bridge 用相邻两帧 `RobotState.header.stamp` 差即时算出（`bridge_node.cpp::on_state`）。

### 5.5 sensor

线格式：`uint32 LE 头长度` + `JSON 头` + 二进制 payload，编解码实现于 `rus_sim_utils/protocol.hpp`
（`EncodeSensorFrame` / `DecodeSensorFrame`）。bridge 订阅 `/sensor/pointcloud`（`sensor_topic`）
后调 `EncodeSensorFrame` → `WsServer::BroadcastSensor`，**一帧 = 一条 WS 二进制消息**，
覆盖式（只留最新一帧，慢客户端丢帧）。帧可到兆级，故该通道必须**动态分配缓冲**并走
`LWS_WRITE_BINARY`（照抄 `/state` 的定长 16 KiB 写法会静默截断）。

> ⚠️ **覆盖式通道的推送必须按「代次」幂等**：lws 对同一次 `lws_callback_on_writable()`
> 请求会**重复回调** `LWS_CALLBACK_SERVER_WRITEABLE`（实测一个 service 周期内可回调数百次）。
> 若回调里无条件 `lws_write`，同一份数据会被重发数百次——`/state` 通道曾因此被放大到
> **1.7 万条/s**（实测：发布 10 Hz 却推 59685 条/3.5s）。现由 `state_gen_` / `sensor_gen_`
> 与 `SessionInfo::{state,sensor}_sent_gen` 保证同一版本每会话只推一次。

---

## 6. 指令总表（以代码为准）

指令名常量：`rus_sim_utils/command_defs.hpp::CmdName`；
参数校验：`rus_sim_utils/command_types.hpp`（结构体 `ParseArgs`，无 `ParseArgs` 的指令**不接受任何参数**）；
路由：`rus_sim_bridge/src/rus_sim_bridge/command_dispatcher.cpp::init_routing()`。

### 6.1 bridge 本地处理（不转发下游）

| 指令 | args | 行为 | result |
|------|------|------|--------|
| `shutdown` | 无 | 先回 reply，再 `rclcpp::shutdown()` | 空 |
| `set_mode` | `[mode]` | 改路由模式：0=手动（直控 driver）、1=自动（planning 协调，默认）。切手动时额外向 planning 发一条 `stop` | 空 |

### 6.2 → PLANNING（`/planning/command`）

| 指令 | args 校验 | result | 说明 |
|------|----------|--------|------|
| `set_start_pose` | ≥3（3=位置 / 6=位置+RPY / 7=位置+四元数） | 空 | `MakePose()` |
| `set_end_pose` | ≥3（同上） | 空 | `MakePose()` |
| `plan` | 无 | 空 | 预扫查门 + 起终点门；成功发 `plan_done`，失败发 `error` |
| `execute` | 无 | 空 | movel 到起点 → `servo_start` → 125Hz 下发 `servo_cart`；结束/中止发 `scan_done` |
| `pause` | 无 | 空 | 停伺服下发并标记暂停（`query_motion_done` 仍返回 0） |
| `resume` | 无 | 空 | 从暂停位置继续 |
| `reset` | ≤2（`[mode?, enable?]`） | 空 | 停伺服 + `stop` + 清轨迹 |
| `query_motion_done` | 无 | `[0/1]` | 执行中/暂停中 = 0，空闲 = 1 |

### 6.3 → DRIVER（`/driver/command`）

| 指令 | args 校验 | result | 说明 |
|------|----------|--------|------|
| `connect` / `disconnect` | 无 | 空 | 用参数 `robot_ip` |
| `is_connected` | 无 | `[0/1]` | |
| `is_in_drag_teach` | 无 | `[状态]` | |
| `robot_enable` | ≥1 | 空 | 1=使能，0=去使能 |
| `get_state` | 无 | `[timestamp, q1..q6]` | |
| `is_motion_done` | 无 | `[0/1]` | |
| `switch_driver` | ≥1 | 空 | `[type, ip1,ip2,ip3,ip4]`；type 0=sim、1=real；不足 5 个则沿用当前 `robot_ip` |
| `movej` | ≥6 | 空 | `[q1..q6, speed?, acc?]` |
| `movel` | ≥3 | 空 | `[x,y,z(,rx,ry,rz)(,speed,acc)]`；目标为 **TCP** 位姿，工具变换由驱动内部处理 |
| `servoj` | ≥6 | 空 | 关节伺服 |
| `servo_cart` | ≥6 | 空 | 笛卡尔伺服（planning 内部按 125Hz 调用） |
| `start_jog` | ≥5 | 空 | `[ref, axis, dir, speed%, acc%, max_dis?]`，speed/acc 为百分比（内部 /100） |
| `stop_jog_decel` / `stop_jog_immediate` | 无 | 空 | |
| `servo_start` / `servo_end` | 无 | 空 | 伺服模式开关 |
| `run_file` | 无 | 空 | 执行 `script_path` 指定的指令文件 |
| `set_time_speed` | ≥1 | 空 | 仅仿真驱动 |
| `get_time_speed` / `get_sim_time` / `get_frame_rate` | 无 | `[值]` | 仿真驱动；`get_frame_rate` 真实驱动下占位返回 125.0 |
| `step_once` | 无 | 空 | 仅仿真驱动 |

> 手动模式（`set_mode [0]`）下 `pause` / `resume` / `reset` / `query_motion_done` 也路由到 DRIVER；
> `stop` 手动模式**只**发 DRIVER，自动模式扇出 `{PLANNING, DRIVER}`。

### 6.4 → PERCEPTION（`/perception/command`）

bridge 注册表登记的是 `pre_scan_start` / `pre_scan_end` / `query_prescan_done`，
但 `perception_node.cpp::handle_command` 实际只实现 `map_clear` 与 `load_cloud`：

| 指令 | 实现方 | bridge 是否注册 | 当前可用性 |
|------|--------|----------------|-----------|
| `map_clear` | perception | ❌ 未注册 | 前端不可达 |
| `load_cloud` | perception（`args[0]`=索引 → `pcd_dir` 下第 N 个 `.pcd`；无参 → `input_pcd`） | ❌ 未注册 | 前端不可达 |
| `pre_scan_start` / `pre_scan_end` | ❌ 未实现 | ✅ 注册 | 转发后返回 `unknown command: pre_scan_*` |
| `query_prescan_done` | ❌ 未实现（planning 侧有实现） | ✅ 注册到 PERCEPTION | 转发后返回 `unknown command` |

### 6.5 扇出指令

| 指令 | 自动模式 | 手动模式 |
|------|---------|---------|
| `stop` | `{PLANNING, DRIVER}` | `{DRIVER}` |

### 6.6 ⚠️ 已定义但**当前不可达**的指令（写文档时必须显式标注）

| 指令 | 实现处 | 不可达原因 |
|------|--------|-----------|
| `pre_scan_done` | planning（`prescan_done_` 门 + 点云初始化） | `command_types.hpp` 无结构体 + bridge 未注册 → 前端发不进来；**连带 `plan` 恒失败** |
| `map_clear` / `load_cloud` | perception | bridge 未注册 + `command_types.hpp` 无结构体 |
| `set_tool_calib_point` / `compute_tool_calib` / `set_tool_coord` / `set_tool_index` / `get_tool_coords` | driver（`cmd_parser.cpp` + `dispatch`） | bridge **已注册**，但 `command_types.hpp` 无对应结构体 → `ParseCommand` 直接判 `unknown command` |

---

## 7. 事件与 ack_id

| 事件名 | 常量 | 发布者 | 触发时机 | ack_id 归属 | success |
|--------|------|--------|---------|------------|---------|
| `plan_done` | `EventName::kPlanDone` | planning | `Plan()` 成功 | `plan` 的 id | true |
| `scan_done` | `EventName::kScanDone` | planning | 伺服执行完毕 / 被 `stop` 中止 | `execute` 的 id | true / false |
| `error` | `EventName::kError` | planning | plan 前置门失败、生成/插值失败 | 触发指令 id | false |
| `pre_scan_done` | `EventName::kPreScanDone` | **无发布者** | —— | —— | —— |
| `motion_done` | `EventName::kMotionDone` | **无发布者**（预留） | —— | —— | —— |

已实现的失败 message：`plan failed: 未完成预扫查` / `plan failed: 起终点未设置` /
`plan failed: 轨迹生成失败` / `plan failed: 生成轨迹为空` / `plan failed: 插值失败`。

链路：planning `publish_event()` → `/module_events` → bridge `on_module_event()`
→ `ResultMessage::MakeEvent(event, client_id, …)` → 广播所有 `/control` 连接。

---

## 8. 参数表（参数名 / 默认值 / 配置文件）

> 「默认值」= 代码 `declare_parameter` 的默认值；「文件值」= 仓库内 yaml 的实际取值（launch 会加载 yaml，故**运行时以文件值为准**）。

### 8.1 `bridge_node`（`config/bridge_params.yaml`）

| 参数 | 类型 | 默认值 | 文件值 | 说明 |
|------|------|--------|--------|------|
| `ws_port` | int | 8765 | 8765 | WS 监听端口，占用则 +1/+2 |
| `state_topic` | string | `/driver/state` | `/driver/state` | 状态流数据源 |
| `sensor_topic` | string | `/sensor/pointcloud` | `/sensor/pointcloud` | 感知流数据源（`SensorFrame`，→ `/sensor`） |
| `forward_sensor` | bool | true | true | 是否转发感知流（false = 不订阅，通道无数据） |
| `timeout_ms` | int | 5000 | 5000 | 下游服务调用超时 |
| `drain_ms` | int | 10 | 10 | 指令队列出队周期 |

### 8.2 `planning_node`（`config/planning_params.yaml`）

| 参数 | 类型 | 默认值 | 文件值 | 说明 |
|------|------|--------|--------|------|
| `point_cloud_topic` | string | `/preprocessed_cloud` | 同左 | 规划输入点云 |
| `driver_state_topic` | string | `/driver/state` | 同左 | 机械臂状态 |
| `driver_command_service` | string | `/driver/command` | 同左 | 伺服指令转发目标 |
| `servo_rate_hz` | double | 125.0 | 125.0 | 伺服下发频率 |
| `interpolate_points` | int | 10 | 10 | 相邻路径点插值点数 |
| `movel_timeout_sec` | double | 10.0 | 10.0 | execute 前 movel 到起点超时 |
| `alpha` | double | 1.0 | 1.0 | 椭圆 Gabriel 条件参数 |
| `graph_k` / `normal_k` / `projection_k` | int | 30 | 30 | 建图 k-NN / 法线 / 重投影 |
| `tol` | double | 1e-6 | 1e-6 | 长度变化容差 |
| `max_iter` | int | 40 | 40 | 最大迭代轮次 |
| `use_smoothing` | bool | true | true | Taubin 平滑开关 |
| `lambda` / `mu` | double | 0.63 / −0.65 | 同左 | Taubin 平滑参数 |
| `end_hold_sec` | —— | **未参数化** ⚠️ | —— | 终点保持时长，`planning_node.hpp` 成员初值 1.0s，yaml 无法覆盖 |

### 8.3 `perception_node`（`config/perception_params.yaml`）

| 参数 | 类型 | 默认值 | 文件值 |
|------|------|--------|--------|
| `source` | string | `auto` | `realsense` | 数据源：`auto` / `realsense` / `ros_topic` / `replay`（空值按 `auto` 处理） |
| `input_cloud_topic` | string | `/camera/camera/depth/color/points` | 同左 | 仅 `source=ros_topic` 使用 |
| `output_cloud_topic` | string | `/preprocessed_cloud` | 同左 |
| `frame_topic` | string | `/perception/frame` | 同左 |
| `sensor_cloud_topic` | string | `/sensor/pointcloud` | 同左 |
| `driver_state_topic` | string | `/driver/state` | 同左 |
| `mapping_mode` | string | `rolling` | `rolling` | 建图模式：`none`（不建图，发当前帧）/ `rolling`（累积 + 上限降采样）/ `accumulate`（只累积不降采样） |
| `rs_serial` | string | `""` | `""` | RealSense 序列号（空 = 第一台；多相机必填） |
| `rs_width` / `rs_height` / `rs_fps` | int | 640 / 480 / 15 | 同左 | 深度/彩色流配置（型号不支持时自动退回设备默认配置） |
| `rs_align_to` | string | `none` | `color` | 点云光学系：`color`（可取 RGB，同旧 `/camera/.../points`）/ `none` / `depth`；**必须与 `camera_to_flange` 标定同系** |
| `rs_color_mode` | string | `rgb` | `rgb` | `rgb`（需 `rs_align_to=color`，否则自动降级）/ `white` |
| `rs_point_stride` | int | 2 | **1** | 像素抽稀步长（点数 ≈ 1/stride²）；效果对齐阶段 = 1（全分辨率，≈30.7 万点/帧） |
| `rs_min_depth` / `rs_max_depth` | double | 0.15 / 6.0 | 同左 | 有效深度区间（米），区间外显式剔除；旧文件值 2.0 过窄会裁掉场景（D435 有效最近距≈0.105m，下限不产生可见裁剪） |
| `rs_decimation` / `rs_spatial_filter` / `rs_temporal_filter` | bool | false | false | SDK 抽稀 / 空间 / 时间滤波；**要贴合 realsense-viewer 应优先开这三项**（viewer 的 Post-Processing 同一套算法），而非 PCL 侧 voxel/SOR |
| `replay_path` | string | `""` | `""` | `source=replay` 的 PCD 文件或目录 |
| `replay_fps` / `replay_loop` | double / bool | 5.0 / true | 同左 | 回放频率 / 是否循环 |
| `replay_frame_id` | string | `camera_optical_frame` | 同左 | 回放帧坐标系名（诊断用） |
| `max_allowed_diff_sec` | double | 0.05 | 0.05 |
| `max_pose_cache` | int | 256 | 256 |
| `process_period` | double | 0.1 | 0.1 |
| `allow_stale_pose` | bool | false | false |
| `map_publish_period` | double | 2.0 | 2.0 |
| `map_max_points` | int | 500000 | 500000 |
| `input_pcd` | string | `""` | `""` | 启动即加载的 PCD（base_link 系场景，非相机帧；空 = 不加载） |
| `pcd_dir` | string | `""` | `""` |
| `enable_passthrough_filter` | bool | true | **false** | 直通滤波开关（效果对齐阶段关闭） |
| `enable_voxel_filter` | bool | true | **false** | 体素降采样开关（效果对齐阶段关闭） |
| `enable_statistical_filter` | bool | false | false | 统计滤波开关（KDTree 开销大，高频处理建议关） |
| `voxel_leaf_size` | double | 0.005 | 0.005 | ≤ 0 = 未配置（跳过该阶段）；**rolling 触发 `map_max_points` 时地图仍按此值降采样** |
| `passthrough_field` | string | `z` | `z` | 直通字段 x / y / z |
| `passthrough_limit_min` / `_max` | double | −0.5 / 0.5 | 同左 | base_link 系 ROI（**不是相机 FOV**，须按真实作业空间重设） |
| `passthrough_negative` | bool | false | false | |
| `statistical_mean_k` | int | 50 | 50 |
| `statistical_std_dev_mul` | double | 1.0 | 1.0 |
| `camera_to_flange` | double[16] | 单位阵（未设置时） | 已标定矩阵（行优先） |

### 8.4 `driver_node`（`config/driver_params.yaml`）

| 参数 | 类型 | 默认值 | 文件值 | 说明 |
|------|------|--------|--------|------|
| `driver_type` | string | `"sim"` | **`"real"`** ⚠️ | sim / real |
| `robot_ip` | string | `""` | `192.168.58.2` | connect / switch_driver 用 |
| `script_path` | string | `""` | `""` | `run_file` 的指令文件 |
| `tool_coords` | double[] | `[0,0,0,0,0,0]` | 12 个值（2 组） | 每 6 个一组 [x,y,z,rx,ry,rz]，0=法兰 |
| `tool_index` | int | 0 | 1 | 当前工具坐标系索引 |
| `tool_coords_file` | string | `~/.rus_sim/tool_coords.yaml` | `""`（即用默认） | 工具坐标系持久化文件 |

### 8.5 `recorder_node`（`config/recorder_params.yaml`）

> 记录文件格式与通道号约定见 `docs/Protocol/RecFormat.md`。

| 参数 | 类型 | 默认值 | 文件值 | 说明 |
|------|------|--------|--------|------|
| `enabled` | bool | true | true | false = 不订阅、不落盘（只留一行日志） |
| `output_dir` | string | `records` | `records` | 输出目录（相对路径按启动工作目录解析，不存在则创建；日志打印绝对路径） |
| `file_prefix` | string | `run` | `run` | 文件名 `<prefix>_<时间戳>.rusrec`；滚动加 `_p001/_p002` |
| `max_file_size_mb` | int | 512 | 512 | 单文件上限，超出即封存（写尾索引）并滚动；0 = 不限 |
| `flush_interval_sec` | double | 1.0 | 1.0 | 写缓冲落盘周期（被强杀最多丢这一段） |
| `queue_max` | int | 2048 | 2048 | 写队列条数上限（满则丢新 + 计数，绝不阻塞发布端） |
| `queue_max_mb` | int | 256 | 256 | 写队列字节上限（防大点云帧吃爆内存） |
| `qos_depth` | int | 20 | 20 | 订阅队列深度（reliable + volatile，与两个发布端兼容） |
| `log_period_sec` | double | 5.0 | 5.0 | 统计日志周期（速率 / 队列 / 丢弃 / 当前文件大小） |
| `record_state` | bool | true | true | 通道 0 开关 |
| `state_topic` | string | `/driver/state` | 同左 | 通道 0 来源 |
| `state_max_rate_hz` | double | 0.0 | 0.0 | 通道 0 写入限流（0 = 全量 125Hz） |
| `record_sensor` | bool | true | true | 通道 1 开关 |
| `sensor_topic` | string | `/sensor/pointcloud` | 同左 | 通道 1 来源 |
| `sensor_max_rate_hz` | double | 0.0 | 0.0 | 通道 1 写入限流（0 = 不限） |

---

## 9. 包内部结构（文件级，仅第一方代码）

### 9.1 `rus_sim_utils`（header-only）

| 文件 | 内容 |
|------|------|
| `command_defs.hpp` | 通道路径、指令名、事件名、感知帧类型常量（纯字符串，无逻辑） |
| `command_types.hpp` | 指令结构体 + `ParseArgs` 校验 + `CommandVariant` + 编译期生成的查找表 + `ParseCommand()` |
| `command_registry.hpp` | `Module` 枚举、`module_service_name()`、`CommandRegistry`（Register / SetTargets / TargetsOf / IsLocal） |
| `protocol.hpp` | `Channel`、`CommandMessage`、`ResultMessage`（MakeReply / MakeEvent）、JSON 序列化/解析、SensorFrame 二进制编解码 |
| `robot_state.hpp` | `ControlTarget`、`RobotState`（通用，无 SDK 依赖） |
| `utils.hpp` | 位姿 ↔ 矩阵、`MakePose`、`PoseToRPY`、`FlangePosToPose`、`FlangeToProbe` / `ProbeToFlange` |

### 9.2 `rus_sim_bridge`

| 文件 | 内容 |
|------|------|
| `include/rus_sim_bridge/bridge_node.hpp` + `src/.../bridge_node.cpp` | 节点：`Create()` 工厂、指令队列、事件/状态转发 |
| `command_dispatcher.{hpp,cpp}` | 解析 → 路由表 → 服务扇出 → 回执聚合 + 超时检测 + 模式切换 |
| `ws_server.{hpp,cpp}` | libwebsockets 多通道服务器（会话注册表 / 覆盖式状态 / 逐会话推送队列） |
| `src/main.cpp` | 入口 |

### 9.3 `rus_sim_driver`

| 目录 / 文件 | 内容 |
|------|------|
| `src/rus_sim_driver/driver_node.cpp` | 节点：参数、工厂建驱动、工具坐标系加载与持久化、服务与话题、125Hz 状态定时器、`dispatch()` 分发 |
| `src/driver/` | `robot_driver.hpp`（接口 `IRobotDriver` + `DriverFactory`）、`sim_driver.{hpp,cpp}`（MuJoCo）、`real_driver.{hpp,cpp}`（Fairino SDK） |
| `src/trajectory/` | `trajectory_executor`、`servo_segment`、`planned_segment`、`jog_segment` |
| `src/controller/` | `controller`、`ctc_controller`、`gravity_comp_controller` |
| `src/components/` | `cmd_parser.cpp`（指令 → 结构体）、`kinematics.cpp`、`types.hpp` |
| `robot_model/` | `fairino3_v6.urdf`、`fairino3_v6_mujoco.urdf`、`robot_mujoco.xml`、`robot_limit.xml`、`convert_urdf_to_mjcf.py`、`meshes/` |
| `scripts/keyboard_control.py` | 键盘点动（随包安装到 `lib/rus_sim_driver/`） |
| `SDK/`、`third_party/EAIK/` | 供应商 SDK 与第三方 IK（不计入第一方代码） |

### 9.4 `rus_sim_planning`

| 文件 | 内容 |
|------|------|
| `planning_node.{hpp,cpp}` | 节点 + 指令门 + 伺服执行状态机（`servo_tick` / 终点保持 / pause / resume / reset）+ 事件发布 |
| `trajectory_generator.{hpp,cpp}` | 点云 → 稀疏路径（建图 / 法线 / Gabriel 条件 / 平滑） |
| `trajectory_interpolator.{hpp,cpp}` | 稀疏路径 → 稠密轨迹 + 当前索引推进 |
| `collision_checker.{hpp,cpp}` | 碰撞检查 |

### 9.5 `rus_sim_perception`

| 文件 | 内容 |
|------|------|
| `perception_node.{hpp,cpp}` | 节点：数据源装配 / 调度 / 发布 + 指令（`map_clear` / `load_cloud`）；点云处理定时器在独立 `MutuallyExclusive` 回调组 |
| `include/camera/` | `point_cloud_source`（数据源接口 + `CloudFrame` 契约）、`frame_slot`（最新帧单槽，覆盖式 + 统计）、`ros_topic_source`（PointCloud2 订阅）、`realsense_source`（librealsense2 直连，pimpl；缺库时由 `realsense_source_stub.cpp` 顶替）、`replay_source`（PCD 循环回放）、`source_factory`（按 `source` 装配） |
| `include/components/` | `pose_interpolator`（125Hz 位姿缓存 + 时间插值）、`sensor_encoder`（SensorFrame 编码）、`types.hpp` |
| `include/pointcloud/` | `spatial_transformer`（相机→base 变换）、`cloud_filter_pipeline`（滤波链 `nan_remove → 直通 → 统计 → 体素`，后三段各有 `enable_*_filter` 开关）、`map_manager`（累积地图 + 上限降采样）、`cloud_io`（PCD 读写） |
| `src/tools/gen_test_cloud.cpp` | 生成 `test_cloud.pcd`（离线联调工具） |

#### RealSense 直连要点（`source=realsense`）

- **时间戳**：必须用 `node->now()`（ROS 时间）。SDK 设备时钟与 ROS 时钟不同源，直接用设备时间会与 `/driver/state` 的位姿时间轴不可比，时间对齐必然失败。
- **无效点**：深度 0（无回波）与超 `rs_min_depth` / `rs_max_depth` 的点在源内显式剔除，不会产出 `(0,0,0)` 鬼点（否则会在 base_link 原点堆出一团点）。
- **点云坐标系**：由 `rs_align_to` 决定 —— `color` → `camera_color_optical_frame`（与旧 `/camera/camera/depth/color/points` 同系，原 `camera_to_flange` 标定可直接沿用）；`none` / `depth` → `camera_depth_optical_frame`。**改 `align_to` 必须同步重做/换算标定**。
- **设备独占**：librealsense2 与 `realsense2_camera` 不能同时打开同一台相机。要与包装节点并存请改 `source=ros_topic`。
- **快速失败**：无设备时 `pipe.start()` 会长时间阻塞且不给可读错误，故启流前先用 `rs2::context::query_devices()` 探测，失败即在 ~0.3s 内返回可操作提示（节点保持存活，其它指令仍可用）。
- **点数控制**：`rs_point_stride`（像素抽稀，直接降点数）与 `rs_decimation`（SDK 降分辨率采样）功能重叠，一般只用其一；点数 ≈ `(W/stride)×(H/stride)`。
- **反投影**：用最终深度帧（对齐 / 抽稀后）的内参 `fx/fy/ppx/ppy` 逐像素反投影，`z = 深度值`；彩色按 `BGR8` 取 R/G/B（仅 `align_to=color` 时像素一一对应）。

#### 效果对齐（与 `realsense-viewer` 对拍）

目标：先排除自研滤波带来的观感差异（"我们发出去的点云"≈"viewer 里的画面"），再逐项开滤波。

- **对照通道用 `/perception/frame`**：无论 `mapping_mode` 取值，它都是"当前帧"（单视角）。
  `rolling` / `accumulate` 下 `/preprocessed_cloud` 与 `/sensor/pointcloud` 是**累积地图快照**（`scope=map`），
  点数远大于单帧，不能拿去跟 viewer 的单帧画面对拍。
- **滤波链逐段开关**：`enable_passthrough_filter` / `enable_statistical_filter` / `enable_voxel_filter`
  （默认 passthrough / voxel 开、statistical 关）。对齐阶段建议三段全关，只留无副作用的 `nan_remove`。
  **不要用 `voxel_leaf_size: 0` 当"关闭"手段**——PCL 内部按 `1/leaf_size` 建索引会除零；实现里已把
  `≤0` 处理为"未配置、跳过"，开关语义仍以 `enable_*` 为准。
- **推进顺序**：① 三段全关看基线 → ② 开 `enable_voxel_filter`（只变稀、几乎不改观感，但直接决定
  `/sensor` 的 WS 带宽）→ ③ 开 SDK 侧 `rs_spatial_filter` / `rs_temporal_filter`（viewer 的
  Post-Processing 面板就是同一套算法，观感最易对上）→ ④ 最后才调 `passthrough_limit_*`
  （`base_link` 系 ROI，**不是相机 FOV**，必须按真实作业空间重设）。
- **临时覆盖 yaml 做 A/B**：`--params-file` 与 `-p` **按出现顺序生效、后者胜**，故 `-p` 要写在
  `--params-file` 之后（否则 yaml 值反过来覆盖命令行）：
  `ros2 run rus_sim_perception rus_sim_perception_node --ros-args --params-file src/rus_sim_perception/config/perception_params.yaml -p enable_voxel_filter:=true`
- 启动日志会打印生效的滤波链（`滤波链：直通=… 统计=… 体素=…`），排查时先看这一行。
- **联调实测**（`test_cloud.pcd`，原始 12120 点）：三段全关 = 12120 点（`/sensor` payload 88 KB，
  `/preprocessed_cloud` 的 `width` 与 `SensorFrame.points` 一致）；`voxel_leaf_size=0.05` → 140 点；
  `passthrough z∈[-0.02, 0]` → 3379 点，且 `range_min/max` 的 z 恰好等于 ROI 边界。

### 9.6 `rus_sim_bringup`

| 文件 | 内容 |
|------|------|
| `launch/rus_sim.launch.py` | 组合 bridge + driver + planning + perception（`record:=true` 时另含 recorder） |
| `scripts/` | `check_pipeline.py`、`exec_monitor.py`、`flange_vis.py`、`movel_test.py`、`movel_single_test.py`、`tool_calib_six_point.py`、`tool_tf_vis.py`、`traj_vis.py`、`run_sensor_tf_rviz.sh`、`test_prescan.sh`（**联调脚本，未在 CMakeLists 中安装**） |

---

### 9.7 `rus_sim_recorder`

| 文件 | 内容 |
|------|------|
| `recorder_node.{hpp,cpp}` | 节点：双路订阅（`/driver/state` + `/sensor/pointcloud`）→ CDR 序列化 → 有界队列 → 独立写线程（flush / 超限滚动 / 封存写尾索引）；参数与统计日志 |
| `include/components/` | `rec_format.hpp`（格式常量 + 结构体 + `static_assert` 尺寸自校验）、`crc32.hpp`（IEEE CRC32，constexpr 查表）、`bin_writer.hpp`（小端缓冲写入器）—— **纯 header / 零 ROS 依赖** |
| `include/storage/` + `src/storage/` | `rec_writer`（`.rusrec` 写：头 + 通道表 + 记录 + 尾索引 + Footer）、`rec_reader`（尾索引读取 / 顺序扫描 / CRC 校验 / 随机定位）—— **纯 std，零 ROS 依赖** |
| `src/tools/recorder_inspect.cpp` | 离线体检 CLI（`--scan` / `--check-crc` / `--dump N` / `--json`），退出码区分"完好 / 打不开 / 有问题" |
| `config/recorder_params.yaml` + `launch/recorder.launch.py` | 参数与启动（§8.5） |

> 分层与 perception 一致（`components/` 基元、`storage/` 领域逻辑、节点只做通信与调度）；
> 记录文件的二进制格式见 `docs/Protocol/RecFormat.md`（含字段级偏移表、崩溃恢复流程、容量估算）。

---

## 10. 命名 / 代码规范现状（与 `DevelopmentGuide.md` 的对照要点）

- **命名空间不统一**：`RusUtils` / `RusSimPlanning` / `RusPerception` / `RusRecorder` / `RusDriverNode` / `RusRobotDriver` / `RusSimRobotDriver` / `rus_sim_bridge`（桥接是全小写，与规范里「`Rus` 开头」不符）；driver 内并存 `RusRobotDriver` 与 `RusSimRobotDriver`。
- **库命名**：各包均为 `<域>_core`（`bridge_core` / `driver_core` / `planning_core` / `perception_core`）且为 `SHARED`；节点可执行只含 `src/main.cpp`，链接自己的 core 库。
- 命名细则（类大驼峰、成员变量 `_` 结尾、`on_*` / `publish_*` 回调、`sub_` / `pub_` / `server_` / `client_` 成员后缀、`kPascalCase` 常量）在现有代码中大体遵守，可原样保留为正式规范。

---

## 11. 已知「文档 ↔ 代码」不一致清单（本轮核对结果）

| # | 位置 | 文档说法 | 代码事实 | 处置建议 |
|---|------|---------|---------|---------|
| 1 | `docs/README.md` 项目结构 | 列出 `frcobot_ros2 / rus_sim_controller / rus_sim_force / rus_sim_planner / rus_sim_pointcloud / rus_sim_task_executor` | 实际只有 7 个包（含 `rus_sim_bridge` / `rus_sim_driver` / `rus_sim_perception` / `rus_sim_planning`） | 按 §1.1 重写 |
| 2 | `docs/README.md` 依赖 | libigl / MoveIt / pinocchio 及固定版本号 | 无这些依赖；实际为 libwebsockets / zstd / PCL / Boost.graph / MuJoCo / EAIK / Fairino SDK | 按 §1.3 重写 |
| 3 | `README.md` | 「快速开始」为空代码块；两个文档链接都指向 `docs/README.md` | —— | 补构建/启动命令，区分前后端文档入口 |
| 4 | `WsProtocol.md` §4.2 | `pre_scan_start` / `pre_scan_end` / `query_prescan_done` → PLANNING | bridge 路由到 **PERCEPTION**，且 perception 未实现这些指令 | 先定契约（§12-1），再改文档 |
| 5 | `WsProtocol.md` §4.4 | 「PERCEPTION 指令待设计，暂不定义」 | 与 §4.2 自相矛盾；perception 已实现 `map_clear` / `load_cloud` | 统一为真实状态 |
| 6 | `WsProtocol.md` §5 / §6 | `pre_scan_done` 事件在 `pre_scan_end` 时触发、ack=pre_scan_end | **代码中无任何 `pre_scan_done` 事件发布者**；planning 只发 `plan_done` / `scan_done` / `error` | 删除或按新契约重写 |
| 7 | `WsProtocol.md` §4.3 | `movel` 需 ≥6 参数 | `command_types.hpp` `MoveL::ParseArgs` 接受 ≥3 | 改为「≥3（3=仅位置，姿态保持）」 |
| 8 | `WsProtocol.md` §4.3 | 工具坐标系 5 条指令列为可用 | bridge 已注册但 `command_types.hpp` 无结构体 → 实际返回 `unknown command` | 补 `command_types` 或文档标注不可用 |
| 9 | `WsProtocol.md` §8 | 代码对照写 `src/command_dispatcher.cpp` | 实际为 `src/rus_sim_bridge/command_dispatcher.cpp`；且缺 perception / driver 对照 | 修路径 + 补全 |
| 10 | `WsProtocol.md` §5 失败场景 | `plan failed: 未完成预扫查（无点云数据）`、`pre_scan_end` 无点云 → `error` | 实际文本为 `plan failed: 未完成预扫查`（无括号）；`pre_scan_end` 由 perception 处理，不会产生该错误 | 按代码文本修正 |
| 11 | `WsProtocol.md` §3.1 | 超时 `message="timeout"` | ✅ 与代码一致 | 保留 |
| 12 | `DevelopmentGuide.md` CMake 模板 | `add_library(pointcloud_core STATIC ...)`、库名 `package_core` | 实际全部 `SHARED`、库名 `<域>_core`；节点可执行只含 `main.cpp`；`ament_target_dependencies` 与 `target_link_libraries` 有明确分工 | 用 `bridge`/`planning` 的真实 CMake 重写模板 |
| 13 | `DevelopmentGuide.md` 测试章节 | `option(ENABLE_TEST)` + `test/src/*.cpp` + `--test` 模式 | 7 个包均无 `test/` 目录，`main.cpp` 也未解析 `--test` | 删除或标为「规划中，尚未落地」 |
| 14 | `DevelopmentGuide.md` 目录结构 | 只有 `src/<pkg>/` + `include/<pkg>/` | perception 另有 `include/components` / `include/pointcloud`；driver 有 `src/driver` / `src/trajectory` / `src/controller` / `src/components` | 补充「算法子层目录」约定 |
| 15 | `DevelopmentGuide.md` 命名空间 | 统一 `Rus` 前缀 | 见 §10（有例外与并存命名） | 规范加例外说明或改代码 |
| 16 | `docs/DocExample.md` | 示例内容且 `docs/` 下无 `rus_sim_*/` 包文档 | 例文与实际节点无关 | 明确为模板，并按 §9 生成 7 个包文档 |
| 17 | 参数默认值 | 文档未提 | `end_hold_sec`（planning）未参数化；`driver_params.yaml` 的 `driver_type: "real"` 与代码默认 `"sim"` 不同 | 文档写清「默认值 vs 文件值」差异 |
| 18 | 代码注释（非文档） | `command_dispatcher.cpp:36` 注释称「planning 通过订阅 **pre_scan_done 事件**获取完成标记」 | planning 实际订阅的是 `/preprocessed_cloud`；事件方向是 planning **发布**事件、bridge 订阅。注释与实现相反 | 修注释（避免后续文档照抄错） |

---

## 12. 待决策 / 下一步

1. **预扫查流程归属（阻塞性）**：`pre_scan_start` / `pre_scan_end` / `pre_scan_done` / `query_prescan_done` 归 planning 还是 perception？
   当前状态是「谁也走不通」，`plan` 永远返回失败。需先定契约，再同步改代码与文档。
2. **工具坐标系指令链路**：是否把 5 条工具指令补进 `command_types.hpp`（driver 侧已全实现，只差这一环）。
3. ✅ **`/sensor` 通道**（已落地）：bridge 订阅 `/sensor/pointcloud` → `/sensor` 通道按二进制帧下发
   （覆盖式，参数 `sensor_topic` / `forward_sensor`）。**阶段性决策：暂不处理 `scope=map`**——
   前端只按单帧语义实现（整帧替换），收到快照时走同一条路径兜底（不 merge、不崩），
   不做快照抽帧 / 降采样 / 增量渲染。注意默认 `mapping_mode: rolling` 下发的仍是快照；
   若要真·单帧 + 10 Hz 须切 `mapping_mode: none`，代价是 `planning` 输入一起退化为单视角
   （同一 `publish_cloud()` 出口，见 `perception_params.yaml::mapping_mode` 注释）。
   剩余待定：前端是否默认连该通道。
4. **`map_clear` / `load_cloud`**：是否注册到 bridge（换场景 / 离线回放联调需要）。
5. **坐标语义**：`set_start_pose` / `set_end_pose` 注释为「法兰系」，而 driver 的 `movel` / `servo_cart` 目标为 **TCP**；planning 送入插值器的状态取 `tool_pose`（TCP）。规范里必须明确一条链上的坐标系约定。
6. **文档结构定稿**：`docs/DevelopmentGuide.md`（规范 + 契约总入口）→ `docs/Protocol/WsProtocol.md`（前端协议细则）→ `docs/<pkg>/<pkg>.md`（按 `DocExample.md` 模板生成 7 个包文档）。
7. **本稿合入方式**：评审通过后，把 §1–§10 作为 `DevelopmentGuide.md` 正式内容；§11 转为「文档维护清单」或拆分 issue 消项。
8. **记录文件的离线回放**：`rus_sim_recorder` 已能录 + 校验（`rus_sim_recorder_inspect`），
   但**还不能回放**。下一步建议做 `rus_sim_recorder_replay`：按 `.rusrec` 的时间轴把
   通道 0/1 重新发布回 `/driver/state`、`/sensor/pointcloud`（`RecReader::ReadAt` 已支持
   按尾索引随机定位 + 校验），用于无设备复现问题与前端联调。
   需先定的两个口径：**是否按原时间戳重发 / 是否支持倍速与暂停**、**与真机驱动同时在线的冲突处理**
   （回放的 `/driver/state` 会与真驱动撞话题，须用 `--ros-args -r` 重映射或先停驱动）。

---

> **本稿定位**：第一轮快速初稿。§1–§9 的接口 / 参数 / 路由均已逐条对照代码核验（依据为各节标注的源码文件）。
> **尚未逐行核对**：`trajectory_generator` / `trajectory_interpolator` / `collision_checker` 的算法内部、
> `sim_driver` / `real_driver` 的运动学与控制细节、`rus_sim_bringup/scripts/*` 各脚本用法
> —— 这些属于「实现内部」，不属于对外契约；如需要一并写入规范，请在评审时指出。
