# rus_sim_bridge

> 前后端**唯一**的 WebSocket 网关：把前端指令翻译成 ROS 2 服务调用，把 ROS 话题推成前端数据流。

## 功能概述

- ✅ WS 三通道：`/control`（指令 / 回执 / 事件）、`/state`（机械臂状态流）、`/sensor`（压缩点云流）
- ✅ 指令路由表 + 多模块扇出 + 超时回执聚合（纯转发，无业务逻辑）
- ✅ 模式切换：自动（默认，planning 协调）/ 手动（直控 driver）
- ✅ 事件桥接：`/module_events` → `/control` 广播

> 状态标记：✅ 已完成 🚧 进行中 📝 待办 ❌ 废弃

## 结构

```
rus_sim_bridge/
├── include/rus_sim_bridge/
│   ├── bridge_node.hpp        # 节点：指令队列、话题 → WS 转发、事件桥接
│   ├── command_dispatcher.hpp # 指令解析 → 路由表 → 服务扇出 → 回执聚合 + 超时
│   └── ws_server.hpp          # libwebsockets 多通道服务器（会话表 / 覆盖式推送 / 代次幂等）
├── src/
│   ├── main.cpp
│   └── rus_sim_bridge/        # bridge_node.cpp / command_dispatcher.cpp / ws_server.cpp
├── config/bridge_params.yaml
└── launch/bridge.launch.py
```

## 节点：bridge_node

| 项 | 值 |
|----|----|
| 可执行 | `ros2 run rus_sim_bridge rus_sim_bridge_node` |
| 启动 | `ros2 launch rus_sim_bridge bridge.launch.py` |
| 监听 | `ws://0.0.0.0:8765`（端口被占用自动尝试 +1 / +2） |

### 输入 / 输出

| 方向 | 名称 | 类型 | 说明 |
|------|------|------|------|
| 输入（WS） | `/control` | JSON `command` | `{ id, cmd, args }` |
| 输入（话题） | `/driver/state` | `RobotState` | → `/state` JSON 流（附实算 `frame_rate`） |
| 输入（话题） | `/sensor/pointcloud` | `SensorFrame` | → `/sensor` 二进制帧（`uint32` 头长 + JSON 头 + payload） |
| 输入（话题） | `/module_events` | `ModuleEvent` | → `event` 广播到所有 `/control` 连接 |
| 输出（WS） | `/control` | `reply` / `event` | 回执只发发起连接；事件广播 |
| 输出（WS） | `/state` / `/sensor` | JSON / binary | 覆盖式：只推最新一帧，慢客户端丢帧 |

### 下游服务调用（CommandService）

| 服务 | 覆盖指令 | 备注 |
|------|---------|------|
| `/driver/command` | 驱动 / 运动 / 仿真 / 工具坐标系 | 手动模式下 `stop`、`pause`、`resume`、`reset`、`query_motion_done` 也走这里 |
| `/planning/command` | `set_start_pose` / `set_end_pose` / `plan` / `execute` / `stop` / `pause` / `resume` / `reset` / `query_motion_done` | 自动模式下 `stop` = 扇出 `{PLANNING, DRIVER}` |
| `/perception/command` | `pre_scan_start` / `pre_scan_end` / `query_prescan_done` | ⚠️ 路由已注册但 perception 未实现，转发后返回 `unknown command` |
| `/recorder/command` | `recorder_start` / `recorder_stop` / `recorder_status` | 不随 `set_mode` 切换 |
| `/replayer/command` | 10 条 `replay_*` | 不随 `set_mode` 切换 |

### 本地处理（不转发）

| 指令 | 行为 |
|------|------|
| `shutdown` | 先回 reply，再 `rclcpp::shutdown()` |
| `set_mode` | `[mode]`：0 = 手动（直控 driver）、1 = 自动（默认）；切手动时额外给 planning 发一条 `stop` |

> 完整指令清单与 args 校验见 [WsProtocol.md](../Protocol/WsProtocol.md) §4；
> 路由注册与扇出实现在 `src/rus_sim_bridge/command_dispatcher.cpp::init_routing()` / `apply_mode()`。
> 未注册 → `unknown command: <cmd>`；参数不合法 → `invalid args for command: <cmd>`；
> 下游服务未就绪 → `service unavailable: <服务名>`；超时 → `timeout`。

## 参数（`config/bridge_params.yaml`）

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `ws_port` | int | 8765 | WS 监听端口（占用则 +1 / +2） |
| `state_topic` | string | `/driver/state` | 状态流数据源 |
| `sensor_topic` | string | `/sensor/pointcloud` | 感知流数据源 |
| `forward_sensor` | bool | true | false = 不订阅感知流（`/sensor` 通道无数据） |
| `timeout_ms` | int | 5000 | 下游服务调用超时（毫秒） |
| `drain_ms` | int | 10 | 指令队列出队周期（毫秒） |

## 依赖

| 依赖 | 说明 | 状态 |
|------|------|------|
| libwebsockets | WS 服务器（`find_library(WEBSOCKETS_LIB websockets REQUIRED)`） | ✅ |
| `rclcpp` / `rus_sim_interfaces` / `rus_sim_utils` | ROS 与协议定义 | ✅ |

## 注意

- 感知订阅 QoS 必须与发布端匹配（reliable + `transient_local`），否则 DDS 不建通路（一帧都收不到）。
- `/state` / `/sensor` 为覆盖式通道：同一版本每会话只推一次（`*_gen` 代次判定），避免 lws 重复回调造成重发风暴。
