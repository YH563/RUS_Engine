# RUS_Sim 前端通信协议 v0.1（设计稿）

> 目标：前端（Avalonia）与后端之间**只保留两条单向通路**（前端→后端、后端→前端），
> 通路上按消息类型细分。指令有回执、事件有推送、数据流可丢帧。
> 本文档是协议定义，不绑定实现（WS 端口/连接数/内部转发均未定死）。

---

## 1. 总体设计

### 1.1 两条单向通路

```
                    ┌─────────────────────────────┐
   FE → BE (请求)    │                             │
  ─────────────────► │         后端(单一入口)       │
                    │   app Coordinator → 各模块   │
  ◄───────────────── │                             │
   BE → FE (推送)    └─────────────────────────────┘
```

- **通路 A：前端 → 后端（请求）** —— 仅 `command`（统一消息，查询/下发不区分）
- **通路 B：后端 → 前端（推送）** —— `reply` / `event` / `state`

> **统一原则**：前端所有主动动作一律是 `command`。后端 `dispatch(cmd, args, result)`
> 本就是统一签名——查询指令只是 `result` 里带数据，操作指令 `result` 为空，
> 两者在协议层无差别，由前端按 cmd 名解析 result 含义。

### 1.2 消息类型总表

| 类型 | 方向 | 有 id | 有回执 | 语义 |
|------|------|-------|--------|------|
| `command` | FE→BE | ✅ | ✅ | 所有前端主动动作（查询/下发统一） |
| `reply`   | BE→FE | ✅ | — | 对 command 的响应（result 可空） |
| `event`   | BE→FE | ❌ | — | 异步通知（任务完成/错误/状态变化） |
| `state`   | BE→FE | ❌ | — | 高频状态流（可丢帧） |

### 1.3 通道分离建议

数据流（`state`）与指令流（`command`/`reply`/`event`）分离，避免队头阻塞。
WebSocket 同一端口可挂多连接，按路径区分：

| 连接路径 | 承载类型 | 可靠性 |
|----------|----------|--------|
| `/ws/control` | command / reply / event | 可靠（id 关联回执） |
| `/ws/state` | state（关节状态，125Hz 最新值） | 可丢帧 |
| `/ws/sensor` | 图像 / 点云（预留） | 可丢帧 |

> 单一端口 + 多连接。只有未来需要按端口分流（负载均衡/防火墙）时才考虑独立端口。

---

## 2. 消息格式（JSON）

### 2.1 通路 A：前端 → 后端（仅 command）

```json
// 查询（result 会带数据）
{ "type": "command", "id": 1, "cmd": "is_motion_done", "args": [] }
// 下发（result 为空）
{ "type": "command", "id": 2, "cmd": "movej", "args": [0.1,0.2,0.3,0,0,0, 0.5, 0.5] }
```

字段：
| 字段 | 类型 | 说明 |
|------|------|------|
| `type` | string | `command` |
| `id` | uint64 | 客户端自增，用于关联 reply |
| `cmd` | string | 指令名（见 §3 指令清单） |
| `args` | double[] | 参数数组（可为空） |

### 2.2 通路 B：后端 → 前端

```json
// 响应（对应 command 的 id；result 按 cmd 含义解析，可为空）
{ "type": "reply", "id": 1, "success": true,  "message": "ok", "result": [1.0] }
{ "type": "reply", "id": 2, "success": false, "message": "unknown command", "result": [] }

// 异步事件（无 id）
{ "type": "event", "event": "scan_done",   "data": { } }
{ "type": "event", "event": "error",       "data": { "code": 3, "msg": "..." } }
{ "type": "event", "event": "motion_done", "data": { } }

// 高频状态流（无 id，可丢帧）
{ "type": "state", "timestamp": 1234.5, "frame_rate": 125.0,
  "joint_pos": [...], "joint_vel": [...], "joint_acc": [...],
  "effort": [...], "flange_pos": [...] }
```

字段：
| 字段 | 类型 | 说明 |
|------|------|------|
| `type` | string | `reply` / `event` / `state` |
| `id` | uint64 | 仅 `reply` 有，与请求关联 |
| `success` / `message` | bool / string | 仅 `reply` |
| `result` | double[] | 命令返回数据（仅 `reply` 且 success；查询类带数据，操作类为空） |
| `event` | string | 事件名（仅 `event`） |
| `data` | object | 事件附带数据（可选） |
| 状态字段 | double[] | 仅 `state` |

---

## 3. 指令清单（三层合并去重）

> 分类规则：
> - **所有前端主动动作统一为 `command`**，均有 `reply` 回执
> - 查询类指令：reply 的 `result` 携带数据；操作类指令：`result` 为空
> - **长任务**：ack 后异步完成 → `command` + 完成 `event`
> - 底层指令由 app 协调器转发，**前端通常只直接发高层指令**（底层列表保留供调试/高级模式）
>
> 下表"类型"列已统一为 `command`；`result` 列标注查询类指令的返回值（操作类为空）。

### 3.1 高层指令（前端主用，经 app 协调器路由）

| 指令 | 类型 | args | result | 完成事件 | 路由目标 |
|------|------|------|--------|----------|----------|
| `connect` | command | — | — | — | driver |
| `shutdown` | command | — | — | — | 全系统 |
| `pre_scan_start` | command | — | — | `pre_scan_done` | planning |
| `pre_scan_end` | command | — | — | — | planning |
| `set_start_pose` | command | [x,y,z] | — | — | planning |
| `set_end_pose` | command | [x,y,z] | — | — | planning |
| `plan` | command | — | — | `plan_done` | planning |
| `execute` | command | — | — | `scan_done` | planning |
| `stop` | command | — | — | — | planning + driver |
| `pause` | command | — | — | — | planning + driver |
| `resume` | command | — | — | — | planning + driver |
| `reset` | command | — | — | — | planning |
| `query_prescan_done` | command | — | [0/1] | — | planning |
| `query_motion_done` | command | — | [0/1] | — | driver |
| `record_start` | command | — | — | — | data |
| `record_stop` | command | — | — | — | data |
| `playback_start` | command | — | — | — | data |
| `playback_stop` | command | — | — | — | data |
| `playback_pause` | command | — | — | — | data |
| `playback_resume` | command | — | — | — | data |
| `playback_set_speed` | command | [speed] | — | — | data |
| `playback_seek` | command | [time_s] | — | — | data |
| `playback_step` | command | [direction] | — | — | data |
| `playback_set_loop` | command | [enable] | — | — | data |
| `playback_get_info` | command | — | [cur,total,t_cur,t_total,speed,progress,playing] | — | data |

### 3.2 底层指令（驱动，经 app 转发）

| 指令 | 类型 | args | result |
|------|------|------|--------|
| `movej` | command | [q1..q6, speed?, acc?] | — |
| `movel` | command | [x,y,z,rx,ry,rz, speed?, acc?] | — |
| `servoj` | command | [q1..q6] | — |
| `servo_cart` | command | [x,y,z,rx,ry,rz] | — |
| `start_jog` | command | [ref, axis, dir, speed%, acc%, max_dis?] | — |
| `stop_jog_decel` | command | — | — |
| `stop_jog_immediate` | command | — | — |
| `servo_start` / `servo_end` | command | — | — |
| `stop` / `pause` / `resume` | command | — | — |
| `connect` / `disconnect` | command | — | — |
| `is_connected` | command | — | [0/1] |
| `is_in_drag_teach` | command | — | [0/1] |
| `robot_enable` | command | [state] | — |
| `get_state` | command | [flag] | [timestamp, q1..q6] |
| `is_motion_done` | command | — | [0/1] |
| `run_file` | command | (path 由后端参数配置) | — |
| `switch_driver` | command | [type, ip1..ip4] | — |
| `set_time_speed` | command | [speed] | — |
| `get_time_speed` | command | — | [speed] |
| `get_sim_time` | command | — | [t] |
| `step_once` | command | — | — |
| `get_frame_rate` | command | — | [hz] |

### 3.3 事件清单（后端 → 前端，通路 B）

| 事件 | 触发 |
|------|------|
| `pre_scan_done` | 预扫查完成 |
| `plan_done` | 轨迹规划完成 |
| `scan_done` | 正式扫查完成 |
| `motion_done` | 当前运动完成 |
| `error` | 模块错误（带 code + msg） |
| `recording_started` / `recording_stopped` | 录制状态变化 |
| `playback_started` / `playback_stopped` | 回放状态变化 |

---

## 4. 待定 / 开放问题

1. **前端是否直接发底层指令**：协议支持，但建议默认只暴露高层指令，底层指令仅在"高级模式/调试"下开放。
2. **长任务的进度**：扫查进行中是否需要周期性进度事件（如 `scan_progress`）？当前只有完成事件。
3. **图像/点云通道**：预留 `/ws/sensor`，但格式（base64 / 二进制帧）未定，建议后续数据量大时走二进制分帧。
4. **错误码表**：`event.error` 的 `code` 需要统一编号，待模块实现时补充。
5. **当前现状与目标的差距**：app 协调器 WsServer 尚未实现；driver(8765)/data(8766) WS 为现有实现，需统一到本协议。
