# RUS_Sim 前端通信协议 v0.2

> 目标：前端（Avalonia）与后端之间通过 WebSocket 通信。bridge 为**纯网关**，
> 只做解析 / 路由 / 转发，不含业务逻辑。本协议是前后端唯一的接口契约，
> 前端按本协议实现解析器即可，无需关心后端子模块划分。
>
> **版本要点**
> 1. **指令名统一平铺**：不区分高层 / 底层指令，全部是字符串，路由由后端 bridge 决定。
> 2. **回执统一**：`reply` 与 `event` 同构（`success` / `message` / `result`），
>    前端用同一个解析器处理；事件用 `ack_id` 关联触发它的指令。
> 3. **通道分离**：指令流 / 状态流 / 感知流走不同 WebSocket 路径，避免互相阻塞。
> 4. **数据精度**：所有 double 数值 JSON 输出保留 **6 位小数**。

---

## 1. 通道（三个 WebSocket 连接）

同一端口（默认 8765），前端按路径建立连接：

| 连接路径 | 承载内容 | 可靠性 | 用途 |
|----------|----------|--------|------|
| `/control` | `command` / `reply` / `event` | 可靠（id 关联回执） | 指令收发、事件通知 |
| `/state` | `state`（关节状态高频流） | 可丢帧（只发最新值） | 状态可视化 |
| `/sensor` | 感知二进制帧（预留） | 可丢帧 | 点云 / 图像 |

前端至少连 `/control`。`/state`、`/sensor` 按需连接；**未连接的通道不会收到数据**。

---

## 2. 通路 A：前端 → 后端（command）

前端所有主动操作（查询 / 下发）统一为一条 `command` 消息。

```json
{ "id": 1, "cmd": "is_motion_done", "args": [] }
{ "id": 2, "cmd": "movej", "args": [0.1, 0.2, 0.3, 0, 0, 0, 0.5] }
```

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `id` | uint32 | ✅ | 客户端自增，用于关联 reply / event 的 ack_id |
| `cmd` | string | ✅ | 指令名（见 §4 指令清单） |
| `args` | double[] | ✅（可空数组） | 参数数组 |

说明：

- 后端解析时只读取 `id` / `cmd` / `args` 三个字段，忽略其他字段；
  前端可自行决定是否带 `"type":"command"`。
- 每条指令都**必有 reply**（成功或失败）。
- 参数数量不足 → reply `success=false, message="invalid args for command: <cmd>"`。
- 未知指令 → reply `success=false, message="unknown command: <cmd>"`。
- 无参指令若携带非空 args → 同样判定为 invalid args。
- JSON 无法解析（缺 `cmd` 等）→ reply `success=false, message="malformed command"`，`id=0`。

---

## 3. 通路 B：后端 → 前端

### 3.1 统一回执：reply / event（同构）

两者字段一致，仅语义不同：

- `reply` —— 对 command 的**同步**应答（查询结果 / 校验失败 / 下发结果）
- `event` —— 子模块的**异步**通知（长任务完成 / 错误），由 bridge 订阅 `/module_events`
  topic 转发而来

```json
// reply：成功 / 失败
{ "type": "reply", "id": 1, "success": true,  "message": "ok", "result": [1.0] }
{ "type": "reply", "id": 2, "success": false, "message": "unknown command: xxx", "result": [] }

// event：异步完成通知（id 固定 0，ack_id 关联原指令）
{ "type": "event", "id": 0, "ack_id": 3, "event": "pre_scan_done",
  "success": true, "message": "", "result": [1.0] }
{ "type": "event", "id": 0, "ack_id": 3, "event": "error",
  "success": false, "message": "planning: scan failed", "result": [] }
```

| 字段 | 类型 | reply | event | 说明 |
|------|------|-------|-------|------|
| `type` | string | ✅ | ✅ | `reply` / `event`（按此分支解析） |
| `id` | uint32 | ✅ | 固定 `0` | reply 关联 command id；event 恒为 0 |
| `ack_id` | uint32 | ❌ | ✅ | 触发该事件的 command id（0 = 模块自发，无关联） |
| `event` | string | ❌ | ✅ | 事件名（见 §5） |
| `success` | bool | ✅ | ✅ | 是否成功 |
| `message` | string | ✅ | ✅ | 错误描述 / 附加说明（成功可为空串） |
| `result` | double[] | ✅ | ✅ | 处理结果（查询类带数据，操作类为空数组） |

**路由语义（重要）**：

- `reply` 只回发给**发起该指令的那条 `/control` 连接**。
- `event` 广播给**所有已连接的 `/control` 连接**（无论谁触发的）。

**长任务闭环**：前端发 `command(id=3, cmd="pre_scan_start")` →
后端立即回 `reply(id=3, success=true)` → 子模块完成后 bridge 推
`event(ack_id=3, event="pre_scan_done", ...)`。前端可用 ack_id 把事件挂回原请求。

**超时**：指令转发到子模块后，默认 **5000ms** 无响应 → bridge 回
`reply(success=false, message="timeout")`。

### 3.2 状态流：state（/state 通道）

```json
{ "type": "state", "timestamp": 1234.5, "frame_rate": 125.0,
  "joint_pos": [...], "joint_vel": [...], "joint_acc": [...],
  "effort": [...], "flange_pos": [...] }
```

| 字段 | 类型 | 说明 |
|------|------|------|
| `type` | string | 固定 `state` |
| `timestamp` | double | 驱动侧仿真时间戳（秒） |
| `frame_rate` | double | 帧率（bridge 按相邻两帧时间差计算） |
| `joint_pos` / `joint_vel` / `joint_acc` / `effort` | double[] | 6 维关节数组 |
| `flange_pos` | double[] | 法兰位姿（平移 + 旋转，长度 6） |

- 覆盖式推送：bridge 只保留**最新一帧**，慢客户端丢帧，前端须容忍帧不连续。
- 数组长度固定为 6（关节 1~6）。

### 3.3 感知流：sensor（/sensor 通道，预留）

感知数据（点云 / 图像）为二进制帧，独立于 JSON 通路。

**帧格式**：`uint32 LE 头长度` + `JSON 头` + `二进制 payload`

```
┌────────────────┬───────────────────┬─────────────────┐
│ uint32 LE      │ JSON header       │ binary payload  │
│ (头字节数)      │ (元数据，UTF-8)    │ (点云/图像裸数据) │
└────────────────┴───────────────────┴─────────────────┘
```

JSON 头示例：

```json
// 点云
{ "type": "pointcloud", "points": 100000, "fields": ["x","y","z","intensity"],
  "dtype": "float32", "timestamp": 1234.5, "seq": 1024 }
// 图像
{ "type": "image", "width": 1920, "height": 1080, "encoding": "rgb8",
  "step": 5760, "timestamp": 1234.6, "seq": 1024 }
```

| 头字段 | 类型 | 适用 | 说明 |
|--------|------|------|------|
| `type` | string | 全部 | `pointcloud` / `image` / `compressed`（预留） |
| `timestamp` | double | 全部 | 时间戳 |
| `seq` | uint32 | 全部 | 帧序号（前端检测丢帧） |
| `points` | uint32 | 点云 | 点数 |
| `fields` | string[] | 点云 | 分量顺序（如 x,y,z,intensity） |
| `dtype` | string | 点云 | 数值类型（float32 / float64 / uint8 / int32） |
| `width` / `height` | uint32 | 图像 | 宽高（像素） |
| `encoding` | string | 图像 | rgb8 / bgr8 / mono8 ... |
| `step` | uint32 | 图像 | 每行字节数（含 padding） |

> 该通道目前**尚无数据源**，前端可先不实现；实现时按头独立解析，不依赖连续帧。

---

## 4. 指令清单（按路由分组）

> 指令名、参数校验、路由目标以后端代码为准（`rus_sim_utils` + bridge 注册表）。
> 下表按 bridge 当前路由分组，仅用于前端参考。

### 4.1 本地指令（bridge 直接处理，不下发子模块）

| 指令名 | args | result | 说明 |
|--------|------|--------|------|
| `shutdown` | 无 | 空 | 关闭整个系统。reply 成功后再退出 |

### 4.2 路由到 PLANNING（规划）

| 指令名 | args | result | 说明 |
|--------|------|--------|------|
| `pre_scan_start` | 无 | 空 | 预扫查开始（完成后有 `pre_scan_done` 事件） |
| `pre_scan_end` | 无 | 空 | 预扫查结束 |
| `set_start_pose` | [x, y, z] | 空 | 设置起点（≥3 个参数） |
| `set_end_pose` | [x, y, z] | 空 | 设置终点（≥3 个参数） |
| `plan` | 无 | 空 | 开始规划（完成后有 `plan_done` 事件） |
| `execute` | 无 | 空 | 开始执行（完成后有 `scan_done` 事件） |
| `query_prescan_done` | 无 | [0/1] | 查询预扫查是否完成 |

### 4.3 路由到 DRIVER（驱动）

| 指令名 | args | result | 说明 |
|--------|------|--------|------|
| `connect` | 无 | 空 | 连接机器人 |
| `disconnect` | 无 | 空 | 断开连接 |
| `is_connected` | 无 | [0/1] | 是否已连接 |
| `is_in_drag_teach` | 无 | [0/1] | 是否拖拽示教中 |
| `robot_enable` | [state] | 空 | 使能（1）/ 去使能（0） |
| `get_state` | 无 | [timestamp, q1..q6] | 获取当前关节状态 |
| `is_motion_done` | 无 | [0/1] | 运动是否完成 |
| `switch_driver` | [type, ip1, ip2, ip3, ip4] | 空 | 切换 sim(0) / real(1)，IP 四个十进制段 |
| `movej` | [q1..q6, speed?, acc?] | 空 | 关节运动（≥6 个参数） |
| `movel` | [x,y,z,rx,ry,rz, speed?, acc?] | 空 | 笛卡尔直线运动（≥6 个参数） |
| `servoj` | [q1..q6] | 空 | 关节伺服（≥6 个参数） |
| `servo_cart` | [x,y,z,rx,ry,rz] | 空 | 笛卡尔伺服（≥6 个参数） |
| `start_jog` | [ref, axis, dir, speed%, acc%, max_dis?] | 空 | 开始点动（≥5 个参数） |
| `stop_jog_decel` | 无 | 空 | 点动减速停止 |
| `stop_jog_immediate` | 无 | 空 | 点动立即停止 |
| `servo_start` | 无 | 空 | 伺服模式开始 |
| `servo_end` | 无 | 空 | 伺服模式结束 |
| `run_file` | 无 | 空 | 执行指令文件（路径由后端参数配置） |
| `set_time_speed` | [speed] | 空 | 设置仿真时间倍速 |
| `get_time_speed` | 无 | [speed] | 查询倍速 |
| `get_sim_time` | 无 | [t] | 查询仿真时间 |
| `step_once` | 无 | 空 | 单步仿真 |
| `get_frame_rate` | 无 | [hz] | 查询帧率 |

### 4.4 扇出指令（同时路由到 PLANNING + DRIVER）

多目标指令：bridge 并发转发给所有目标，**全部成功才返回 success**，
`result` 按目标顺序拼接；任一失败则 `message` 为首个失败信息。

| 指令名 | args | result |
|--------|------|--------|
| `stop` | 无 | 空 |
| `pause` | 无 | 空 |
| `resume` | 无 | 空 |
| `reset` | 无 | 空 |
| `query_motion_done` | 无 | [0/1] |

### 4.5 PERCEPTION 指令

通道 / 注册已预留，指令待后续设计，暂不定义。感知大块数据（点云 / 图像）
始终走 `/sensor` 二进制通道，不通过 command 传输。

---

## 5. 事件清单

| 事件名 | 触发时机 | 关联指令 | result |
|--------|----------|----------|--------|
| `pre_scan_done` | 预扫查完成 | `pre_scan_start` | 可空 |
| `plan_done` | 轨迹规划完成 | `plan` | 可空 |
| `scan_done` | 正式扫查完成 | `execute` | 可空 |
| `motion_done` | 当前运动完成 | 任意运动指令 | 可空 |
| `error` | 模块错误 | 可空 | 空 |

---

## 6. 前端实现要点（Avalonia）

1. **三个连接**：`/control`（必连）、`/state`（状态可视化）、`/sensor`（预留）。
2. **两个解析器**：
   - JSON 解析器：读 `type` 字段分流 → `reply` / `event`（同构，可复用字段绑定）/
     `state`；
   - 二进制解析器（sensor 预留）。
3. **reply 与 event 同构**：建议建模为一个类（`type` / `id` / `ack_id` / `event` /
   `success` / `message` / `result`），reply 时 `event` 字段为空，event 时 `id` 为 0。
4. **请求追踪**：`id` 自增；reply 按 `id` 匹配；event 按 `ack_id` 挂回原请求。
5. **丢帧容忍**：`/state` 只保留最新值，不要依赖连续性。
6. **数值精度**：result 内所有 double 为 6 位小数字符串，解析成 double 即可。

---

## 7. 代码对照

| 关注点 | 头文件 |
|--------|--------|
| 通道路径 / 指令名 / 事件名 / 传感器类型常量 | `rus_sim_utils/command_defs.hpp` |
| 消息结构体 / JSON 编解码 / 传感器帧 | `rus_sim_utils/protocol.hpp` |
| 指令结构体与参数校验 | `rus_sim_utils/command_types.hpp` |
| 指令 → 模块路由配置 | `rus_sim_utils/command_registry.hpp` + bridge `command_dispatcher.cpp` |
| bridge 实际路由表 | `rus_sim_bridge/src/command_dispatcher.cpp` `init_routing()` |
