# rus_sim_recorder

> 记录层：把机械臂状态与感知点云双通道落盘成 `.rusrec`，并支持把录音按时间轴重发回话题（离线复盘）。

## 功能概述

- ✅ 录制：双通道（`/driver/state` + `/sensor/pointcloud`）异步写盘，不阻塞数据流
- ✅ 运行期开关：`recorder_start` / `recorder_stop` / `recorder_status`（外部可控）
- ✅ 文件管理：按大小滚动（`_pNNN`）、写缓冲周期落盘、写失败熔断
- ✅ 离线回放：按录音时间轴重发原话题，支持倍速 / 暂停 / seek / 单步
- ✅ 体检工具：离线校验头 / 记录 / CRC / 尾索引，损坏文件可扫描恢复

> 状态标记：✅ 已完成 🚧 进行中 📝 待办 ❌ 废弃

## 结构

```
rus_sim_recorder/
├── include/storage/            # rec_writer / rec_reader（纯 std，不依赖 ROS）
├── include/components/         # rec_format（头 / 记录 / 索引布局）、bin_writer、crc32
├── include/rus_sim_recorder/recorder_node.hpp
├── include/rus_sim_replayer/replay_node.hpp
├── src/rus_sim_recorder/recorder_node.cpp   # 录制节点实现
├── src/rus_sim_replayer/replay_node.cpp     # 回放节点实现
├── src/storage/                             # rec_writer.cpp / rec_reader.cpp
├── src/main.cpp                             # → rus_sim_recorder_node
├── src/replay_main.cpp                      # → rus_sim_recorder_replay
├── src/tools/recorder_inspect.cpp           # → rus_sim_recorder_inspect
├── config/recorder_params.yaml
├── config/replayer_params.yaml
└── launch/recorder.launch.py / replayer.launch.py
```

编译产物：库 `rec_storage`（纯 std）/ `recorder_core` / `replay_core`；可执行 `rus_sim_recorder_node`、
`rus_sim_recorder_replay`、`rus_sim_recorder_inspect`。

## 组件 1：recorder_node（录制）

| 项 | 值 |
|----|----|
| 可执行 | `ros2 run rus_sim_recorder rus_sim_recorder_node` |
| 启动 | `ros2 launch rus_sim_recorder recorder.launch.py [autostart:=false] [enabled:=false] [output_dir:=…]` |
| 服务 | `/recorder/command`（`CommandService`） |
| 落盘 | `<output_dir>/<prefix>_<时间戳>.rusrec`（滚动加 `_pNNN`），默认 `records/` |

### 输入 / 输出

| 方向 | 名称 | 类型 | 说明 |
|------|------|------|------|
| 输入（话题） | `/driver/state` | `RobotState` | 通道 0（125 Hz 全量，可 `state_max_rate_hz` 限流） |
| 输入（话题） | `/sensor/pointcloud` | `SensorFrame` | 通道 1（压缩点云，可 `sensor_max_rate_hz` 限流） |
| 输入（服务） | `/recorder/command` | `CommandService` | `recorder_start` / `recorder_stop` / `recorder_status` |
| 输出 | — | — | **只订阅、不发布**（旁路模块），话题流不受录制影响 |

### 指令（`/recorder/command`）

| 指令 | args | result | strings | 说明 |
|------|------|--------|---------|------|
| `recorder_start` | 无 | 7 项状态 | [新文件名] | 开始录制：一律打开**新文件**（`_pNNN` 递增，绝不覆盖） |
| `recorder_stop` | 无 | 7 项状态 | [已封存文件名] | 先排空队列再封存（写尾索引 + Footer），返回即可回放 / 体检 |
| `recorder_status` | 无 | 7 项状态 | [当前 / 最后文件名] | 查询（前端"录制中"指示与计时数据源） |

> 状态编码 `state`：0 = `stopped`、1 = `recording`、2 = `failed`（写失败熔断，需重启节点）；
> 7 项 result 字段顺序与录制口径见 [WsProtocol.md §4.8](../Protocol/WsProtocol.md)。

### 参数（`config/recorder_params.yaml`）

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `enabled` | bool | true | false = 不订阅、不落盘 |
| `autostart` | bool | true | **启动即录**；false = 起来待命，等 `recorder_start` |
| `output_dir` | string | `records` | 输出目录（相对路径按启动工作目录解析） |
| `file_prefix` | string | `run` | 文件名前缀 |
| `max_file_size_mb` | int | 512 | 单文件大小上限，超出滚动；0 = 不限 |
| `flush_interval_sec` | double | 1.0 | 写缓冲落盘周期（被强杀最多丢这一段） |
| `queue_max` / `queue_max_mb` | int | 2048 / 256 | 写队列上限（满则丢新 + 计数，绝不阻塞发布端） |
| `qos_depth` | int | 20 | 订阅队列深度（reliable + volatile） |
| `log_period_sec` | double | 5.0 | 统计日志周期 |
| `record_state` / `state_topic` / `state_max_rate_hz` | bool / string / double | true / `/driver/state` / 0 | 通道 0 开关 / 话题 / 限流（0 = 不限） |
| `record_sensor` / `sensor_topic` / `sensor_max_rate_hz` | bool / string / double | true / `/sensor/pointcloud` / 0 | 通道 1 开关 / 话题 / 限流（0 = 不限） |

## 组件 2：replayer_node（离线回放）

| 项 | 值 |
|----|----|
| 可执行 | `ros2 run rus_sim_recorder rus_sim_recorder_replay` |
| 启动 | `ros2 launch rus_sim_recorder replayer.launch.py [file_index:=1] [topic_prefix:=/replay]` |
| 服务 | `/replayer/command`（`CommandService`） |
| 发布 | 录制时的话题原样重发（`topic_prefix` 非空则加前缀）；另发 `/module_events` 的 `replay_done` |

### 输入 / 输出

| 方向 | 名称 | 类型 | 说明 |
|------|------|------|------|
| 输入 | 录音文件 | `.rusrec` | 由参数 `record_dir` / `file_path` / `file_index` 决定，**不接受前端传路径** |
| 输入（服务） | `/replayer/command` | `CommandService` | 10 条 `replay_*` |
| 输出（话题） | `/driver/state`、`/sensor/pointcloud`（或 `<prefix>…`） | `RobotState` / `SensorFrame` | 原字节重发（**不重打时间戳、不改 payload**） |
| 输出（话题） | `/module_events` | `ModuleEvent` | `replay_done`（播到末尾且 `loop=false`）；读盘 / CRC 失败发 `error` |

### 指令（`/replayer/command`）

| 指令 | args | 说明 |
|------|------|------|
| `replay_list` | 无 | 列出录音文件（`strings` = 文件名清单，文件名升序即时间顺序） |
| `replay_load` | [序号?] | 载入录音（载入后 `state=2`、游标归零）；无尾索引的崩溃录音同样可载入 |
| `replay_start` | [倍速?] | 开始播放（未载入时按启动参数自动载入） |
| `replay_pause` / `replay_resume` | 无 | 暂停 / 从暂停处继续（不跳时间） |
| `replay_stop` | 无 | 停止并复位到起点（文件保留，可直接再 start） |
| `replay_seek` | [t] | 跳到相对文件起点 `t` 秒 |
| `replay_set_speed` | [speed] | 设置倍速（越界钳位到 0.05 ~ 20，播放中即时生效） |
| `replay_step` | [n?] | 单步发布 n 条（缺省 1；仅 paused / idle 可用） |
| `replay_status` | 无 | 查询状态（9 项 result：`state` / `progress` / `duration` / `speed` / `cursor` / `records` / `loaded` / `file_index` / `file_count`） |

> 状态编码：0 = idle、1 = playing、2 = paused、3 = finished；完整字段顺序与典型时序见
> [WsProtocol.md §4.7](../Protocol/WsProtocol.md)。

### 参数（`config/replayer_params.yaml`）

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `record_dir` | string | `records` | 录音目录（与 recorder 的 `output_dir` 对齐即可直接回放） |
| `file_path` / `file_prefix` / `file_index` | string / string / int | `""` / `""` / 0 | 文件选择：显式路径 → 前缀过滤 → 升序第 N 个 |
| `autoload` / `autoplay` / `loop` | bool | true / false / false | 启动即载入 / 载入后即播 / 播完循环 |
| `speed` | double | 1.0 | 初始倍速（0.05 ~ 20） |
| `check_crc` | bool | true | 发布前逐条校验 payload CRC32（失败即停 + `error` 事件） |
| `use_recv_time` | bool | false | 时间轴基准：false = 消息时间戳 `stamp`；true = 录制入队时刻 `recv` |
| `topic_prefix` | string | `""` | 发布话题前缀（`""` = 原样发回；**与真机共存时会撞话题**，隔离用 `/replay`） |
| `qos_depth` / `qos_transient_local` | int / bool | 10 / true | 与 driver / perception 发布端匹配（bridge 要求 `transient_local`） |
| `log_period_sec` | double | 5.0 | 回放统计日志周期 |

## 工具：rus_sim_recorder_inspect（离线体检）

```bash
# 校验完整性（含逐条 CRC）
ros2 run rus_sim_recorder rus_sim_recorder_inspect records/run_20260926_141530.rusrec --check-crc

# 崩溃 / 截断文件（无尾索引）：扫描恢复
ros2 run rus_sim_recorder rus_sim_recorder_inspect records/run_*.rusrec --scan
```

不依赖 ROS 图（只链 `rec_storage`），可直接运行
`install/rus_sim_recorder/lib/rus_sim_recorder/rus_sim_recorder_inspect`。

## 启动 / 常用

```bash
ros2 launch rus_sim_recorder recorder.launch.py                    # 默认启动即录到 records/
ros2 launch rus_sim_recorder recorder.launch.py autostart:=false   # 待命，等 recorder_start
ros2 launch rus_sim_recorder replayer.launch.py                    # 回放 records/ 中第 0 个文件
```

## 依赖

| 依赖 | 说明 | 状态 |
|------|------|------|
| 标准库 | `rec_storage`（文件读写 / CRC32 / 字节序处理）不依赖 ROS | ✅ |
| `rclcpp` / `rus_sim_interfaces` / `rus_sim_utils` / Eigen3 | 节点侧（Eigen3 来自 `rus_sim_utils` 的导出接口） | ✅ |

## 注意

- 录制是**旁路**：只订阅、不发布；停录期间到达的消息不入队、不计 `dropped`。
- 回放默认发回录制时话题：真机在线时**先停驱动**，或用 `topic_prefix:=/replay` 隔离。
- 记录文件格式、通道号约定、容量估算见 [RecFormat.md](../Protocol/RecFormat.md)。
