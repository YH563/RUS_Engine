# RUS_Engine × RUSTool 指令接口对齐计划

> 范围：后端（`RUS_Engine`，本仓库）与前端（`RUSTool`）之间的 **WebSocket 指令/通道契约**对齐。
> 原则：**以后端现有协议为基准**（`docs/Protocol/WsProtocol.md` + `command_defs.hpp`），
> 前端对齐为主；后端只在**过渡期**提供低风险兼容（可选，加性）。
> 目标：前端"预扫查 → 设起终点 → 规划 → 执行"闭环可跑通。

## 1. 不对齐清单（现状）

| 编号 | 现象 | 后端 | 前端 | 严重度 |
|---|---|---|---|---|
| A1 | 预扫查命令名/归属不一致 | `pre_scan_done` → PLANNING（P0 门）；`pre_scan_start/end` → PERCEPTION（未实现） | `EndPreScan` 发 `pre_scan_end`，**从不发 `pre_scan_done`** | **阻断** |
| A2 | 起终点命令缺参数 | `set_start_pose/end` 要求 `args ≥ 3` | `SetStartPoseAsync/EndPoseAsync` **不传 args** | **阻断** |
| A3 | 通道不一致 | `/control /state /sensor /mesh /pcmap` | 仅 `/control /state /sensor` | 中 |
| A4 | 感知帧类型不一致 | `SensorType`：`pointcloud/image/mesh/ultrasound/compressed` | `SensorTypes`：仅 `pointcloud/image/compressed` | 低 |
| A5 | 错误码未消费 | reply/event 带 `error_code` | `ReplyOrEvent` 无该字段 | 中 |
| A6 | 指令常量缺登记 | 含 `map_clear/load_cloud/工具标定 5 条` | `Commands` 未登记 | 低 |

已对齐（无需改动）：`/control` 指令请求字段、reply/event 字段、state 字段、事件名、`scope`/`encoding`、`recorder_*`、回放废弃（两侧一致）。

## 2. 目标契约（对齐后）

- **预扫查**：前端在半自动建图完成后发 **`pre_scan_done`**（→ PLANNING，初始化轨迹生成器并置门）→ 之后 `plan` 才放行。
- **起终点**：前端"记录当前位姿"时，读 `state.tool_pose`（`[x,y,z,rx,ry,rz]`，m/rad）作为 `args` 发 **`set_start_pose` / `set_end_pose`**。
- **规划/执行**：`plan` → `plan_done` 事件；`execute` → `motion_done`/`scan_done` 事件。
- **通道**：前端按需连接 `/mesh`（增量网格，可靠有序）与 `/pcmap`（面元点云图，覆盖式；线格式同 `/sensor`）。
- **错误码**：reply/event 的 `error_code` 由前端解析并分支。

## 3. 后端任务（本仓库）—— 已执行（采用"后端兼容"路径）

> 决策：采用 **后端兼容（B1/B2）**，使前端既有流程（发 `pre_scan_start/pre_scan_end`、无参起终点）直接可用。

- [x] **B1（兼容）**：`pre_scan_start` / `pre_scan_end` 路由到 PLANNING；`pre_scan_start` 空操作确认，
  `pre_scan_end` 等价 `pre_scan_done`。
  - 文件：`src/rus_sim_bridge/src/rus_sim_bridge/command_dispatcher.cpp`（`init_routing`）、
    `src/rus_sim_planning/src/rus_sim_planning/planning_node.cpp`（`handle_command` 分支）。
- [x] **B2（兼容）**：`set_start_pose` / `set_end_pose` **无参**时用当前 TCP 位姿
  （`interpolator_.LatestState().flange_pos`，即 `/driver/state` 的 `tool_pose`）；有参（≥3）仍按参数。
  - 文件：`planning_node.cpp`（`resolve_pose` lambda + 两个分支）。
- [x] **B3（文档）**：`WsProtocol.md` §4.2/§4.4/§4.6、`rus_sim_bridge.md`、`rus_sim_planning.md` 已同步兼容行为。
- [x] **B4（保持）**：`/mesh`、`/pcmap` 通道、`SensorType.mesh`、reply/event `error_code` 保持不变（前端来对齐）。

## 4. 前端任务（RUSTool，外部自行对齐）

- [ ] **F1（阻断）**：补 `Commands.PreScanDone = "pre_scan_done"` 与 `PreScanDoneAsync()`；`EndPreScan` 改发 **`pre_scan_done`**（→planning）。
  - 文件：`RUSTool.Core/Communication/ProtocolConstants.cs`、`RUSTool.Core/Services/Robot/{IRobotService,RobotService}.cs`、
    `RUSTool.UI/ViewModels/ScanWorkflowViewModel.cs`。
  - 若不做 F1，则依赖后端 B1（发 `pre_scan_end` 即视为完成）。
- [ ] **F2（阻断）**：`SetStartPoseAsync/SetEndPoseAsync` 传入当前 `state.tool_pose`（`[x,y,z,rx,ry,rz]`）作为 `args`。
  - 文件：`RobotService.cs`（`SetStartPoseAsync/SetEndPoseAsync`）、`ScanWorkflowViewModel.cs`（调用处取 `LatestState.frame/ tool_pose`）。
  - 若不做 F2，则依赖后端 B2。
- [ ] **F3（中）**：`Channels` 增 `Mesh="/mesh"`、`PcMap="/pcmap"`；`ConnectionManager`/`BridgeClient` 增连接与接收；`SensorTypes.Mesh="mesh"`。
- [ ] **F4（中）**：解码——`/pcmap` 复用 `SensorFrameCodec`（type=pointcloud, scope=map）；`/mesh` 新 `MeshFrameCodec`（→ `MeshChunkUpdate`/`MeshSink`，块 upsert/remove + revision）。
- [ ] **F5（中）**：`BridgeProtocol.ReplyOrEvent` 增 `ErrorCode`（uint32），解析并按码分支。
- [ ] **F6（低）**：`Commands` 补 `map_clear` / `load_cloud` / 工具标定 5 条（若前端要调用）。

## 5. 执行顺序与验收

1. **阻断项 A1/A2**：先定"前端改（F1/F2）"还是"后端兼容（B1/B2）"（建议前端改齐；后端兼容仅作过渡）。
2. **A3/A4/A5**：前端补通道/类型/错误码（F3/F4/F5）。
3. **A6**：按需登记（F6）。

验收（前端 `NextStep` 全流程）：
`pre_scan_start`（或跳过）→ **`pre_scan_done`** → `set_start_pose`/`set_end_pose`（带 tool_pose）→ `plan`（`plan_done`）→ `execute`（`motion_done`/`scan_done`），全程无 `error_code≠0`。
可选链路：连 `/pcmap` 看到面元点云图；连 `/mesh` 看到增量网格。

## 6. 决策与状态
- **D1（已定）**：A1/A2 走 **后端兼容（B1/B2）** —— 已实现，前端既有流程可直接跑通（前端**无需**再改 F1/F2）。
- D2（前端）：A3/A4/A5（`/mesh`/`/pcmap` 通道、`SensorType.mesh`、`error_code`）由前端自行对齐；
  不做不影响现有画面，仅影响"接口对齐"完整度。
