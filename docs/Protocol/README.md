# 协议文档索引

> 前后端之间、模块之间的接口契约都在本目录。**实现以代码为准**：本目录负责把线格式、
> 字段顺序、状态编码写清楚；代码改动后必须回写这里（见文末维护约定）。

## 本目录

| 文档 | 版本 | 管什么 |
|------|------|--------|
| [WsProtocol.md](./WsProtocol.md) | v0.4 | 前端 ⇄ bridge 的 WebSocket 协议：三通道、`command` / `reply` / `event`、`state` / `sensor` 线格式、指令清单、事件清单、扫查时序、前端实现要点 |
| [RecFormat.md](./RecFormat.md) | v1 | `.rusrec` 记录文件：整体布局、FileHeader、通道表、记录、尾索引、读取流程、回放口径、录制开关口径、容量估算 |

## 速查

### 通道（同一端口，按路径连接）

| 路径 | 内容 | 可靠性 |
|------|------|--------|
| `/control` | `command` / `reply` / `event` | 可靠（id 关联回执） |
| `/state` | 机械臂状态高频流 | 可丢帧（只发最新值） |
| `/sensor` | 感知二进制帧（压缩点云） | 可丢帧（只发最新一帧） |

### 模块服务（bridge 调用，均为 `CommandService`）

| 服务 | 指令组 | 参考小节 |
|------|--------|---------|
| `/driver/command` | 驱动 / 运动 / 伺服 / 仿真 / 工具坐标系 | WsProtocol §4.3 |
| `/planning/command` | 规划与执行 | WsProtocol §4.2 |
| `/perception/command` | 感知（⚠️ 注册与实现未对齐，见 §4.4） | WsProtocol §4.4 |
| `/recorder/command` | `recorder_start` / `recorder_stop` / `recorder_status` | WsProtocol §4.8 |
| `/replayer/command` | 10 条 `replay_*` | WsProtocol §4.7 |
| （bridge 本地） | `shutdown` / `set_mode` | WsProtocol §4.1 |
| （扇出） | `stop`（自动模式 → planning + driver） | WsProtocol §4.5 / §4.6 |

### 事件（`event` 消息 / `ModuleEvent`）

| 事件 | 发布者 | 触发 |
|------|--------|------|
| `plan_done` | planning | `plan` 成功 |
| `scan_done` | planning | 伺服执行完毕 / 被 `stop` 中止 |
| `replay_done` | replayer | 回放播到末尾（`loop=false`） |
| `error` | planning / replayer | 前置门失败、生成失败、读盘 / CRC 失败 |
| `pre_scan_done` / `motion_done` | —（预留，无发布者） | — |

### 线格式速记

```jsonc
// 指令（前端 → 后端，/control）
{ "id": 1, "cmd": "recorder_start", "args": [] }

// 回执 / 事件（后端 → 前端，/control）
{ "type": "reply", "id": 1, "success": true, "message": "ok", "result": [1,0,0,0,0,0,1], "strings": ["run_…rusrec"] }
{ "type": "event", "id": 0, "ack_id": 3, "event": "plan_done", "success": true, "message": "", "result": [] }

// 状态流（/state）：type / timestamp / frame_rate / joint_pos… / flange_pos / tool_index / tool_pose
// 感知流（/sensor）：uint32 LE 头长 + JSON 头 + 二进制 payload（zstd + int16 量化）
// 记录文件：64 B FileHeader + ChannelDesc[] + (40 B RecHeader + payload)* + IndexEntry[] + 32 B Footer
```

数值精度：JSON 中所有 `double` 输出保留 **6 位小数**；单位统一为位置 **m** + 角度 **rad**
（换算只在驱动内部发生，前端不做任何换算）。

## 维护约定

1. **加指令**：`rus_sim_utils/command_defs.hpp` 加常量 → `command_types.hpp` 加结构体与 `ParseArgs`
   → bridge `init_routing()` 注册 → 目标模块 `handle_command` 实现 → 补本目录对应小节表格。
2. **加可选字段**：追加在消息 / `result` 尾部，前端需容忍未知字段（`strings` 就是这样加入的，
   与数值通道 `result` 并存）。
3. **改字段顺序或状态编码**：属破坏性变更，必须升协议版本号（WsProtocol 头部）并在本页登记。
4. **`.rusrec` 格式变化**：走文件内版本号与兼容约定，见 [RecFormat.md](./RecFormat.md) §9。

## 联调工具

| 工具 | 位置 | 用途 |
|------|------|------|
| `probe_sensor.py` | `tools/ws_probe/` | ROS 侧：打印 `/sensor/pointcloud` 与 `/preprocessed_cloud` 元数据 |
| `ws_sensor_check.py` | `tools/ws_probe/` | WS 侧：连 `/sensor` 通道，按线格式解析并校验自洽性 |
| `rus_sim_recorder_inspect` | `rus_sim_recorder` | 录音离线体检（`--check-crc` / `--scan`） |

## 相关文档

- 模块侧接口与参数：[../README.md](../README.md) 的模块索引
- 全系统契约核验稿（含冲突清单）：[../DevelopmentGuide.draft.md](../DevelopmentGuide.draft.md)
- 代码规范：[../DevelopmentGuide.md](../DevelopmentGuide.md)
