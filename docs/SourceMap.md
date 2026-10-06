# RUS_Engine 后端架构 / 源码地图

> 一页看清 RUS-Sim 后端：**8 个 ROS 2 包**、对外只有 `rus_sim_bridge` 一个 WebSocket 出口。
> 读法：先看第 1 节（包与依赖）→ 第 2 节（五条主链）→ 再进对应包（第 3 节）。
> 协议契约见 [Protocol/WsProtocol.md](./Protocol/WsProtocol.md)；逐包细节见各 `docs/rus_sim_*/`。

---

## 1. 总览

### 1.1 包与职责

| 包 | 角色 | 关键产物 | 一句话 |
|---|---|---|---|
| `rus_sim_interfaces` | 接口定义 | `.msg` / `.srv` | 全系统消息/服务唯一来源（rosidl） |
| `rus_sim_utils` | 协议/工具基座 | header-only | 通道/指令/事件常量、指令结构体与解析、路由注册表、位姿工具、error_code |
| `rus_sim_driver` | 驱动层 | `driver_core` + `rus_sim_driver_node` | 机械臂运动学/轨迹/控制：**MuJoCo 仿真** or **Fairino 真机**，发布 `/driver/state` |
| `rus_sim_perception` | 感知层 | `perception_core` + 节点 | 相机取帧 → 位姿对齐 → 变换 → 滤波 → 累积建图 → 发布规划/前端点云 |
| `rus_sim_planning` | 规划层 | `planning_core` + 节点 | 点云表面 → 轨迹生成 → 插值 → 125 Hz 伺服执行 |
| `rus_sim_reconstruction` | 重建层 | `reconstruction_core` + 节点 | 面元融合（SurfelMap）/ 稀疏 TSDF → 融合点云图 + 增量网格 |
| `rus_sim_recorder` | 记录层 | `rec_storage` + 节点/工具 | 双通道落盘 `.rusrec` + 离线体检（回放已移交前端） |
| `rus_sim_bringup` | 启动/联调 | 仅 launch/scripts | 一键拉起 + 联调脚本 |

### 1.2 依赖图（编译期）

```
                 rus_sim_interfaces        rus_sim_utils(header-only)
                        ▲                        ▲
        ┌───────────────┼───────────────┬────────┼───────────────┐
        │               │               │        │               │
   rus_sim_bridge  rus_sim_driver  rus_sim_perception  rus_sim_planning  rus_sim_recorder
                                        ▲
                                        │ (复用均匀重采样 cloud_resampler)
                                 rus_sim_reconstruction
```
- `rus_sim_utils` 是纯头文件 INTERFACE 包，**协议/指令/路由的唯一定义处**。
- 唯一一条"逆向"依赖：`reconstruction → perception`（`cloud_resampler` 已迁入 perception）。
- `rus_sim_bridge` 是**纯网关**（无业务逻辑）；后端子模块之间的调用（如 planning → driver）**直连服务**，不经 bridge。

### 1.3 进程 / 线程模型

| 节点 | 线程 |
|---|---|
| `bridge_node` | libwebsockets 事件循环线程 + ROS executor + dispatcher 定时器；`WsServer` 内部加锁 + `lws_cancel_service` 唤醒 |
| `rus_sim_driver_node` | 控制线程（仿真步进 / 真机周期查询）+ ROS executor |
| `perception_node` | 处理定时器（10 Hz，串行）+ 相机取帧线程（写"最新帧槽"）|
| `planning_node` | ROS executor + 执行期 `servo_tick` 定时器（125 Hz）+ 异步 driver 客户端回调 |
| `recorder_node` | 订阅回调（序列化入队）+ 独立写线程 |
| `reconstruction_node` | ROS executor；面元分片并行用内部线程池 |

---

## 2. 端到端主链

> 端口固定 `ws://…:8765`，通道 `/control` `/state` `/sensor` `/pcmap` `/mesh`。

### 链 A · 指令：前端 → 后端（command/reply/event）
```
前端 --WS /control--> bridge_node(ws_server 收) --> command_dispatcher(解析/路由/扇出/超时)
      --> 各模块 CommandService(/driver|planning|perception|recorder/command)
      --> reply（按 id）+ event（/module_events → bridge → /control 广播）
```

### 链 B · 驱动状态（125 Hz）
```
sim/real driver --> /driver/state(RobotState) --> bridge(/state) / planning / perception / recorder
```

### 链 C · 感知 → 规划/前端/重建
```
相机源(RealSense|ROS|PCD回放) --> frame_slot(最新帧)
  --> perception_node::process_frame:
       pose_interpolator(位姿对齐) → spatial_transformer(base_link)
       → cloud_filter_pipeline(直通/统计/体素/重采样) → map_manager(累积建图)
  --> /perception/frame(当前帧, RViz/重建)
      /preprocessed_cloud(地图快照, **planning 输入**)
      /sensor/pointcloud(SensorFrame 压缩帧, bridge/recorder)
```

### 链 D · 规划 → 执行
```
前端: pre_scan_done → set_start/end_pose → plan → execute
planning_node::handle_command:
  plan    → trajectory_generator(点云 k-NN 图 + Dijkstra + 牛顿优化) → trajectory_interpolator(稠密化) → plan_done
  execute → movel 到起点 → servo_start → 125Hz servo_cart → …→ scan_done
```

### 链 E · 重建 → 前端
```
/perception/frame --> reconstruction_node
  --> ShardedSurfelMap(体素哈希融合) --> /sensor/pcmap(面元点云图, scope=map) --> bridge /pcmap
  --> TsdfVolume(稀疏 TSDF + Marching Tetrahedra) --> /sensor/mesh(块增量) --> bridge /mesh（默认关）
```

### 链 F · 记录（旁路）
```
/driver/state + /sensor/pointcloud --> recorder_node(序列化入队 → 写线程) --> <records_dir>/*.rusrec
   运行期开关：recorder_start / recorder_stop / recorder_status（默认待命，前端触发）
```

---

## 3. 逐包结构与入口

### 3.1 `rus_sim_utils`（协议基座，header-only）
`command_defs.hpp`（WsPath/CmdName/EventName/SensorType/SensorScope）· `command_types.hpp`（指令结构体 + `ParseCommand`）·
`protocol.hpp`（`CommandMessage`/`ResultMessage`/`StateMessage`/`SensorFrame`/`MeshFrame` 编解码）·
`command_registry.hpp`（指令→模块路由表）· `error_codes.hpp` · `robot_state.hpp` · `utils.hpp`（位姿工具 `MakePose` 等）。

### 3.2 `rus_sim_bridge`（网关）
`main.cpp` → `bridge_node.{hpp,cpp}`（订阅 `/driver/state`、`/sensor/pointcloud`、`/sensor/pcmap`、`/sensor/mesh`、`/module_events`；广播 `/state`、`/sensor`、`/pcmap`、`/mesh`、事件）→ `command_dispatcher.{hpp,cpp}`（`init_routing()` 注册表 + 扇出 + 超时；`apply_mode()` 按手动/自动改路由）→ `ws_server.{hpp,cpp}`（libwebsockets 多通道；`/sensor`/`/pcmap` 覆盖式单槽 vs `/mesh` 可靠有序队列）。

### 3.3 `rus_sim_driver`（驱动）
`driver_node.{hpp,cpp}`（`/driver/command` 服务 + `/driver/state` 发布 + `/joint_states`）→ `driver/robot_driver.hpp`（抽象接口）→ `driver/sim_driver.cpp`（MuJoCo）/ `driver/real_driver.cpp`（Fairino SDK，勿细读）→
`components/`：`cmd_parser`（指令解析）、`kinematics`（EAIK FK/IK/Jacobian）、`types`/`command_defs`；
`trajectory/`：`trajectory_executor` + `planned/jog/servo_segment`；`controller/`：`controller`/`ctc`/`gravity_comp`。
> 工具坐标系（探头相对法兰）由驱动统一管理，`planning` 不碰。

### 3.4 `rus_sim_perception`（感知）
`perception_node.{hpp,cpp}`（**核心**：`process_frame()` 与各 `publish_*`）→
`camera/`：`point_cloud_source`（接口）+ `realsense/ros_topic/replay_source` + `source_factory` + `frame_slot`（最新帧槽）；
`components/`：`pose_interpolator`（时间对齐）、`sensor_encoder`（zstd+int16 量化）、`types`；
`pointcloud/`：`spatial_transformer` → `cloud_filter_pipeline`（直通/统计/体素/重采样）→ `map_manager`（累积建图）→ `cloud_resampler`（均匀重采样）。

### 3.5 `rus_sim_planning`（规划）
`planning_node.{hpp,cpp}`（`handle_command` 指令入口 + `Plan()`/`Execute()`/`servo_tick()`/`Pause/Resume/Stop/Reset`）→
`trajectory_generator.{hpp,cpp}`（点云 k-NN 建图 + 椭圆 Gabriel 滤边 + Dijkstra + 定向投影/牛顿优化 + Taubin 平滑）→
`trajectory_interpolator.{hpp,cpp}`（稀疏→稠密：位置 lerp + 姿态 slerp）→ `collision_checker.cpp`（桩，暂未接入）。

### 3.6 `rus_sim_reconstruction`（重建）
`reconstruction_node.{hpp,cpp}`（`OnCloud` 面元融合/TSDF 积分；`PublishReconstruction`/`PublishPcMap`/`PublishMesh`）→
`surfel_map.cpp` + `sharded_surfel_map.cpp`（体素哈希 + 邻域匹配 + 法线/位置 EMA + 置信度；分片并行）→
`tsdf_volume.cpp`（稀疏体素 TSDF + Marching Tetrahedra + 块增量导出）→ `voxel_hash.hpp` / `thread_pool.hpp` / `types.hpp`；
`examples/`（可视化/工具，依赖 GLFW/GLEW/PCL）。

### 3.7 `rus_sim_recorder`（记录）
`recorder_node.{hpp,cpp}`（异步写盘：回调序列化入队 → 写线程 → 滚动/CRC/尾索引；`/recorder/command`）→
`storage/rec_writer.cpp` / `rec_reader.cpp`（`.rusrec` 读写）→ `components/`（`rec_format`/`crc32`/`bin_writer`）→
`tools/recorder_inspect.cpp`（离线体检）。`replay_node` 已废弃（回放移交前端）。

### 3.8 `rus_sim_bringup`（启动）
`launch/rus_sim.launch.py`（组合 bridge/driver/planning/perception/recorder）+ `scripts/`（联调脚本：`test_prescan.sh`/`traj_vis.py`/`check_pipeline.py` 等）。

---

## 4. 对外接口速查

### 4.1 WebSocket 通道
| 路径 | 内容 | 可靠性 |
|---|---|---|
| `/control` | command / reply / event | 可靠 |
| `/state` | 状态高频流 | 覆盖式 |
| `/sensor` | 感知压缩点云帧（SensorFrame） | 覆盖式 |
| `/pcmap` | 面元点云图（SensorFrame，scope=map） | 覆盖式 |
| `/mesh` | 增量网格块（MeshFrame） | 可靠有序（默认关） |

### 4.2 ROS 话题 / 服务
- 话题：`/driver/state`、`/joint_states`、`/perception/frame`、`/preprocessed_cloud`、`/sensor/pointcloud`、`/sensor/pcmap`、`/sensor/mesh`、`/reconstructed_cloud`、`/planned_trajectory`、`/module_events`
- 服务：`/driver/command`、`/planning/command`、`/perception/command`、`/recorder/command`（`/replayer/command` 已废弃）

---

## 5. 关键设计决策（读码时留意）
1. **协议单一来源**：指令/通道/帧类型常量都在 `rus_sim_utils`；加指令只改 `command_defs.hpp` + `command_types.hpp` + bridge 注册 + 目标模块处理。
2. **单位统一**：协议层位置 m + 角度 rad；m↔mm / rad↔deg 换算**只在驱动内部**。
3. **前端与 planning 解耦**：`/sensor/pointcloud` 的 `scope`/`sensor_scope` 决定前端看单帧还是地图，planning 始终用 `/preprocessed_cloud`（地图快照）。
4. **稀疏 TSDF 页/块**：内存 ∝ 表面积（非体积）；`/mesh` 按块 upsert/remove + revision（可靠有序）。
5. **面元 vs TSDF**：`/pcmap`=融合点云图（规划/分割基础），`/mesh`=网格（可视化/未来碰撞）；当前网格默认关。
6. **录制旁路**：只订阅不发布；默认待命，由前端 `recorder_start` 开录；回放移交前端（直读 `<records_dir>/*.rusrec`）。
7. **构建**：部署用 `colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release`（默认无 -O2）。

---

## 6. 读码入口（按链路）
- 指令链路：`bridge_node.cpp` → `command_dispatcher.cpp::init_routing` → 目标节点 `handle_command`。
- 感知链路：`perception_node.cpp::process_frame` → `cloud_filter_pipeline` → `map_manager`。
- 规划链路：`planning_node.cpp::Plan/Execute/servo_tick` → `trajectory_generator` → `trajectory_interpolator`。
- 重建链路：`reconstruction_node.cpp::OnCloud/PublishPcMap/PublishMesh` → `surfel_map` / `tsdf_volume`。
- 记录链路：`recorder_node.cpp` → `rec_writer/rec_reader`（配 `docs/Protocol/RecFormat.md`）。
- 用法范例：各包 `test/*.cpp`（如 `test_tsdf.cpp`、`test_perception.cpp`）。
