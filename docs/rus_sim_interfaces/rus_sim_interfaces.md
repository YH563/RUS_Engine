# rus_sim_interfaces

> 全系统消息 / 服务定义的**唯一来源**：3 个 `.msg` + 1 个 `.srv`。

## 功能概述

- ✅ `RobotState.msg`：机械臂状态（关节 / 法兰 / TCP），125 Hz
- ✅ `SensorFrame.msg`：感知帧（点云 / 图像 / 超声），payload 压缩 + 反量化元数据
- ✅ `ModuleEvent.msg`：子模块事件上报（→ bridge 广播）
- ✅ `CommandService.srv`：统一指令服务接口（五个模块同构复用）

> 状态标记：✅ 已完成 🚧 进行中 📝 待办 ❌ 废弃

## 结构

```
rus_sim_interfaces/
├── msg/
│   ├── RobotState.msg     # header / joint_pos,vel,acc,effort / flange_pos / tool_index / tool_pose
│   ├── SensorFrame.msg    # type / encoding / seq / frame_id / scope / 点云元数据 / data
│   └── ModuleEvent.msg    # event / success / message / result / client_id / module / stamp
├── srv/
│   └── CommandService.srv # 请求 client_id+command+args → 响应 success+message+result+strings
├── CMakeLists.txt         # rosidl_generate_interfaces
└── package.xml
```

## 消息速查

### `RobotState`（发布者：`rus_sim_driver`）

| 字段 | 类型 | 说明 |
|------|------|------|
| `header` | `std_msgs/Header` | 时间戳（驱动 / 仿真时间） |
| `joint_pos` / `joint_vel` / `joint_acc` | `float64[]` | 6 关节 [rad] / [rad/s] / [rad/s²] |
| `effort` | `float64[]` | 关节力矩 [Nm] |
| `flange_pos` | `float64[]` | 法兰位姿 `[x,y,z,rx,ry,rz]`（m / rad） |
| `tool_index` | `int32` | 当前工具坐标系索引，0 = 法兰 |
| `tool_pose` | `float64[]` | 当前 TCP 位姿（基坐标系，XYZABC，m / rad） |

### `SensorFrame`（发布者：`rus_sim_perception`）

| 字段 | 类型 | 说明 |
|------|------|------|
| `type` | `uint8` | `TYPE_POINTCLOUD=0` / `TYPE_IMAGE=1` / `TYPE_ULTRASOUND=2`（后两者预留） |
| `encoding` | `string` | payload 压缩算法：`zstd` / `raw` / `jpeg` / `png` |
| `seq` / `stamp` / `frame_id` | — | 每类型独立递增序号 / 时间戳 / 点云坐标系 |
| `scope` | `string` | 数据语义：`frame`（当前帧）/ `map`（累积地图快照） |
| `points` / `fields` / `dtype` | — | 点数 / 分量顺序（如 `["x","y","z","rgb"]`）/ 量化类型（`int16`） |
| `range_min` / `range_max` | `float64[]` | int16 量化包围盒，**前端反量化必需** |
| `width` / `height` / `step` / `image_encoding` | — | image / ultrasound 元数据（预留） |
| `data` | `uint8[]` | 压缩后 payload |

### `ModuleEvent`（发布者：`rus_sim_planning` / `rus_sim_replayer`）

| 字段 | 类型 | 说明 |
|------|------|------|
| `event` | `string` | `plan_done` / `scan_done` / `replay_done` / `error`（常量见 `rus_sim_utils::EventName`） |
| `success` / `message` / `result` | — | 结果与附加信息 |
| `client_id` | `uint32` | 触发指令的前端 id，用于回关联 `ack_id`（0 = 无关联） |
| `module` | `string` | 来源模块（`driver` / `planning` / `perception`） |
| `stamp` | `builtin_interfaces/Time` | 事件时间 |

### `CommandService`（服务端：五个模块同构）

```text
# 请求
uint32   client_id   # 前端 command id（bridge 透传；0 = 无）
string   command     # 指令名（rus_sim_utils::CmdName）
float64[] args       # 指令参数（是否合法由各模块 / command_types.hpp 决定）
---
# 响应
bool     success
string   message     # 失败原因 / 提示
float64[] result     # 数值结果（查询类）
string[] strings     # 文本结果（如回放的录音文件清单）
```

## 使用方

| 消息 / 服务 | 发布 / 服务端 | 订阅 / 调用端 |
|-------------|--------------|--------------|
| `RobotState` | driver | bridge、planning、perception、recorder |
| `SensorFrame` | perception | bridge、recorder |
| `ModuleEvent` | planning、replayer | bridge |
| `CommandService` | driver / planning / perception / recorder / replayer | bridge（+ planning → driver） |

> 修改任何字段都会同时影响前后端：协议侧同步改 [WsProtocol.md](../Protocol/WsProtocol.md)，
> 前端解码代码与两端的字段索引必须一起更新。

## 依赖

| 依赖 | 说明 | 状态 |
|------|------|------|
| `rosidl_default_generators` | 代码生成 | ✅ |
| `std_msgs` / `builtin_interfaces` / `geometry_msgs` | 字段引用的标准消息 | ✅ |
