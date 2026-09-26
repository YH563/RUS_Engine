# RUS_Sim 记录文件格式 v1（`.rusrec`）

> 目标：把运行时数据流（机械臂状态 + 感知帧）落成**单个自描述文件**，供离线复盘 /
> 回归测试 / 前端复现。写入与读取实现在 `src/rus_sim_recorder/`（`storage/` 为纯 std，
> 不依赖 ROS，可被任何工具复用）。
>
> **版本要点**
> 1. **单文件 + 可流式追加**：`FileHeader → 通道表 → 记录流 → 尾索引 → FileFooter`。
> 2. **每条记录自带魔数 `REC1`**：进程被强杀（无尾索引）的文件仍可**顺序扫描**读回，
>    不会因为丢掉尾部索引而整份文件作废。
> 3. **payload 自带 CRC32**：写入截断 / 磁盘坏块在读取时即被发现，而不是等到回放时
>    解出一堆垃圾点云。
> 4. **通道表声明话题与消息类型名，payload 是 ROS 消息的 CDR 字节**：
>    离线工具按 `type_name` 反序列化即可还原消息（不需要 rosbag2 / 不需要原话题在线）。
> 5. **全部小端、逐字段显式编解码**：不依赖主机字节序与编译器 padding；
>    结构尺寸写进文件头，将来加字段时老工具能按尺寸跳过未知区域。
>
> 参考实现：`storage/rec_writer.cpp`（写）、`storage/rec_reader.cpp`（读）；
> 体检工具：`rus_sim_recorder_inspect`；格式常量：`components/rec_format.hpp`。

---

## 1. 整体布局

```text
┌────────────────┬────────────────────────────┬───────────────────┬────────────────────┬────────────┐
│ FileHeader     │ ChannelDesc × channel_count │ Record × N         │ IndexEntry × N     │ FileFooter │
│ 64 B 固定      │ 变长（u16 长度前缀字符串）  │ 40 B 头 + payload  │ 32 B × N           │ 32 B 固定  │
└────────────────┴────────────────────────────┴───────────────────┴────────────────────┴────────────┘
^ 文件偏移 0                     ↑ 记录流起点（stream_offset）               ↑ index_offset
```

- 记录流起点 = `header_size + 通道表字节数`（通道表长度由内容决定，不固定）。
- `payload` 长度由 `RecHeader.payload_size` 给出；记录之间**无填充字节**。
- 尾索引 + Footer 只在**正常关闭**（含 Ctrl-C）时写入；缺失即视为"未正常关闭"。

---

## 2. FileHeader（偏移 0，固定 64 B）

| 偏移 | 长度 | 字段 | 类型 | 说明 |
|------|------|------|------|------|
| 0 | 8 | `magic` | char[8] | `"RUSRECv1"`（含版本，便于肉眼确认） |
| 8 | 2 | `version` | u16 | 格式版本，当前 `1`；不匹配则拒绝读取 |
| 10 | 2 | `header_size` | u16 | 本结构长度（= 64）；记录流起点由此字段起算 |
| 12 | 2 | `rec_header_size` | u16 | 记录头长度（= 40）；将来加字段时用于跳步 |
| 14 | 2 | `index_entry_size` | u16 | 索引项长度（= 32） |
| 16 | 4 | `flags` | u32 | 预留（当前写 0，读侧忽略） |
| 20 | 8 | `created_unix_ns` | i64 | 录制开始时刻（本机系统时钟，纳秒） |
| 28 | 2 | `channel_count` | u16 | 紧随其后的通道描述条数 |
| 30 | 2 | `reserved` | u16 | 保留（写 0） |
| 32 | 32 | `reserved2` | bytes | 保留（写 0，供将来扩展） |

> `created_unix_ns` 是**墙钟**；记录里的 `stamp_ns` 是**消息时间戳**（ROS 时间）。
> 两者不是同一时基（真机上可能差几个数量级），不要混用。

---

## 3. 通道表（`ChannelDesc` × `channel_count`）

每条通道描述变长，字段顺序固定：

| 字段 | 类型 | 说明 |
|------|------|------|
| `channel_id` | u16 | 通道号（同一文件内唯一；见 §6 约定） |
| `kind` | u16 | payload 编码方式：`0` = `ros_msg`（ROS 消息 CDR） |
| `topic_len` + `topic` | u16 + UTF-8 | 记录来源话题，如 `/driver/state` |
| `type_len` + `type_name` | u16 + UTF-8 | 消息类型，如 `rus_sim_interfaces/msg/RobotState` |
| `note_len` + `note` | u16 + UTF-8 | 人类可读说明（inspect 打印用） |

字符串一律 **u16 长度前缀 + UTF-8 字节**（无 NUL 结尾）。

---

## 4. 记录（`RecHeader` + payload）

### 4.1 RecHeader（固定 40 B）

| 偏移 | 长度 | 字段 | 类型 | 说明 |
|------|------|------|------|------|
| 0 | 4 | `magic` | char[4] | `"REC1"` —— 顺序扫描靠它定位，也是"损坏点"的判定依据 |
| 4 | 2 | `channel_id` | u16 | 所属通道 |
| 6 | 2 | `kind` | u16 | 与通道描述一致（0 = `ros_msg`） |
| 8 | 4 | `seq` | u32 | **通道内**自增（从 0）：跳变 = 丢记录或丢文件段 |
| 12 | 8 | `stamp_ns` | i64 | 消息时间戳（ROS 时间；0 = 消息未带时间戳） |
| 20 | 8 | `recv_ns` | i64 | 入队时刻（本机系统时钟）：`recv_ns - stamp_ns` ≈ 传输 + 排队延迟 |
| 28 | 4 | `payload_size` | u32 | payload 字节数（≤ 256 MiB，读侧防御上限 `kMaxPayloadSize`） |
| 32 | 4 | `payload_crc32` | u32 | payload 的 CRC32（IEEE，zlib 同算法）；`payload_size=0` 时为 0 |
| 36 | 4 | `reserved` | u32 | 保留（写 0） |

### 4.2 payload

`kind = 0`（`ros_msg`）：payload 是 `rclcpp::Serialization<T>` 产出的 CDR 字节
（含 4 字节 CDR encapsulation header），`T` 由通道表的 `type_name` 决定。

- C++ 侧还原：`rclcpp::Serialization<T> ser; ser.deserialize_message(&serialized, msg);`
- Python 侧还原：`rclpy.serialization.deserialize_message(payload, SensorFrame)`
- 点云 payload 内部结构（zstd + int16 量化 + `range_min/max` 反量化）见
  `docs/Protocol/WsProtocol.md` §3.3。

---

## 5. 尾索引与 FileFooter

### 5.1 IndexEntry（32 B × 记录总数）

| 偏移 | 长度 | 字段 | 类型 | 说明 |
|------|------|------|------|------|
| 0 | 8 | `offset` | i64 | 该条**记录头**在文件中的绝对偏移 |
| 8 | 8 | `stamp_ns` | i64 | 与 RecHeader 一致（免读 payload 即可统计时间跨度） |
| 16 | 2 | `channel_id` | u16 | |
| 18 | 2 | `kind` | u16 | |
| 20 | 4 | `payload_size` | u32 | |
| 24 | 4 | `payload_crc32` | u32 | |
| 28 | 4 | `reserved` | u32 | 保留（写 0） |

### 5.2 FileFooter（固定 32 B，文件最后 32 字节）

| 偏移 | 长度 | 字段 | 类型 | 说明 |
|------|------|------|------|------|
| 0 | 8 | `index_offset` | i64 | 索引区起始偏移（= 记录流结束偏移） |
| 8 | 8 | `index_count` | i64 | 索引项条数（= 记录总数） |
| 16 | 8 | `file_size` | i64 | 文件总字节数（含本 Footer）；与真实大小不符 → 索引不可信 |
| 24 | 8 | `magic` | char[8] | `"RUSIDX01"` |

**索引自洽性检查**（读取方应做）：`offset` 严格递增；首条 ≥ 记录流起点；
末条满足 `offset + rec_header_size + payload_size == index_offset`。

---

## 6. 通道号约定

| channel_id | 话题 | 消息类型 | 内容 |
|------------|------|----------|------|
| 0 | `/driver/state` | `rus_sim_interfaces/msg/RobotState` | 机械臂全状态（关节 / 法兰 / TCP），125 Hz |
| 1 | `/sensor/pointcloud` | `rus_sim_interfaces/msg/SensorFrame` | 感知帧（zstd 压缩点云；`scope` 区分当前帧 / 地图快照） |

- 通道号是**离线工具的稳定契约**：只要号不变，老工具读新文件仍然正确。
- 未启用的通道（`record_state` / `record_sensor` = false）**不出现在通道表里**，
  即一个文件可能只有通道 1。
- 新增通道（如 `/perception/frame` 原始 PointCloud2、图像、超声）时**追加新号**，
  不复用旧号、不改旧号语义。

---

## 7. 读取流程

### 7.1 正常文件（有尾索引）

```text
读 FileHeader → 校验 magic / version → 读通道表 → 校验 Footer magic
  → 读索引区 → 校验自洽性（§5.2）→ 统计 / 随机访问（IndexEntry.offset → 该条记录）
```

不读 payload 即可给出记录数、时间跨度、各通道吞吐（`recorder_inspect` 默认路径）。

### 7.2 崩溃 / 截断文件（无尾索引）

```text
读 FileHeader → 读通道表 → 从记录流起点顺序扫描：
  循环：读 40 B 记录头
        magic != "REC1"        → 停止（尾部残留字节数如实报出）
        payload_size 超上限    → 停止（头部损坏）
        读 payload → 可选 CRC 校验 → 前进 rec_header_size + payload_size
```

- 已知尾部残缺（只写了一半的记录）不影响前面所有记录的读取。
- `seq` 跳变说明"中间丢过记录"（队列满 / 写失败）；末尾少记录则只是"最后一段没落盘"
  （写缓冲默认 1 s 落盘一次，被强杀最多丢这 1 s）。

### 7.3 读取工具

```bash
ros2 run rus_sim_recorder rus_sim_recorder_inspect <file.rusrec>              # 索引统计 + 自洽性
ros2 run rus_sim_recorder rus_sim_recorder_inspect <file.rusrec> --check-crc  # 逐条 CRC 校验
ros2 run rus_sim_recorder rus_sim_recorder_inspect <file.rusrec> --scan       # 崩溃文件（自动降级）
ros2 run rus_sim_recorder rus_sim_recorder_inspect <file.rusrec> --dump 5 --json
```

退出码：`0` 完好 / `1` 打不开或头部非法 / `2` 能读但有问题（CRC 失败、截断、索引不一致）。

录制开关（运行期开 / 关落盘，§7.5）：

```bash
ros2 service call /recorder/command rus_sim_interfaces/srv/CommandService "{command: recorder_status}"
```

回放（把录音按时间轴重发回话题，§7.4）：

```bash
ros2 launch rus_sim_recorder replayer.launch.py                    # 载入 records/ 里第 0 个，等 replay_start
ros2 launch rus_sim_recorder replayer.launch.py autoplay:=true     # 启动即播
ros2 launch rus_sim_recorder replayer.launch.py topic_prefix:=/replay   # 与真机共存（话题隔离）
# 前端/命令行控制：replay_list / replay_load / replay_start / pause / resume / stop / seek / status
ros2 service call /replayer/command rus_sim_interfaces/srv/CommandService "{command: replay_status}"
```

### 7.4 回放（`rus_sim_recorder_replay`）

回放把"**什么时候发哪条记录**"与"**这条记录的 payload**"分成两件事：

```text
载入：Open()（读 FileHeader / 通道表；有尾索引则顺手读索引）
  → BuildTimeline()：从记录流起点顺序扫描，只读 40 B 记录头，payload 用 seek 跳过（不读内容）
      → [offset, channel, seq, stamp_ns, recv_ns, payload_size, crc] × N
  → 按通道表建泛型发布器（话题名 / 类型名都来自录音的 ChannelDesc）

回放：睡到时间轴上的点 → ReadRecordAt(offset)（读回 payload + 可选 CRC 校验）→ 原样重发
```

要点：

- **有尾索引与无尾索引（崩溃 / 截断）都能回放**：时间轴来自顺序扫描，不依赖 `IndexEntry`；
  有索引时若索引条数与扫描结果不一致，告警并以扫描结果为准（索引仍是 inspect 体检的随机访问路径）。
- **时间轴基准**（`replayer_node` 参数 `use_recv_time`）：
  - `false`（默认）：`RecHeader.stamp_ns`（消息时间戳）——复现驱动当时的时基；
  - `true`：`RecHeader.recv_ns`（录制入队时刻）——复现录制端的实际到达节奏。
  - 首条平移到 0（进度以文件起点计）；时间戳回退（`map_clear` 复位 / 通道交错）**钳到前一条**，
    保证倍速播放不倒流；`stamp_ns == 0`（消息未带时间戳）时回退用 `recv_ns`。
- **payload 不驻留内存**：时间轴只有元数据（每条 ≈ 32 B），长录音也不会把 GB 级 payload 读进内存；
  发布前一刻才按 `offset` 读回并做 CRC 二次校验（`check_crc` 默认开）。
- **发布方式**：泛型发布器（`rclcpp::GenericPublisher`）+ 录制时的 CDR 字节**原样重发**，
  不反序列化、不重打时间戳 → 订阅端看到的字节与录制时一致；`transient_local` 默认开，
  与 bridge 对 `/sensor/pointcloud` 的订阅要求一致（否则 DDS 不建立通路）。
- **失败处理**：读回 / CRC 失败 → 停止回放并广播 `error` 事件（绝不继续发坏数据）；
  "最后一条写了一半"的记录不进时间轴（扫描在截断处收住，日志给出 `payload 截断 @offset`）。
- **与真机共存**：默认按录制时的话题原样发布，真驱动在线会撞话题 → 先停驱动，
  或用参数 `topic_prefix` 隔离（`/replay` → `/replay/driver/state`）。

### 7.5 录制开关（`/recorder/command`）

录制默认**启动即录**（参数 `autostart=true`），也可由外部指令在运行期开关
（协议细节见 `docs/Protocol/WsProtocol.md` §4.8）：

```bash
ros2 launch rus_sim_recorder recorder.launch.py autostart:=false    # 起来待命，等指令
ros2 service call /recorder/command rus_sim_interfaces/srv/CommandService "{command: recorder_start}"
ros2 service call /recorder/command rus_sim_interfaces/srv/CommandService "{command: recorder_status}"
ros2 service call /recorder/command rus_sim_interfaces/srv/CommandService "{command: recorder_stop}"
```

三条指令都无参；回执 `result` = `[state, 记录数, payload MiB, 当前文件 MiB, 丢弃, 限流, 文件数]`，
`strings` = 当前 / 最后封存的文件名。

| `state` | 含义 |
|---------|------|
| 0 | `stopped`：未录制（可再 `recorder_start`） |
| 1 | `recording`：录制中 |
| 2 | `failed`：写失败熔断（需检查磁盘后重启节点） |

要点：

- **stop 会排空队列**：先把已入队的记录写完，再写尾索引 + Footer（回执返回即代表文件完好，
  可直接 `recorder_inspect` 体检 / `rus_sim_recorder_replay` 回放），**不丢已入队数据**。
- **每次 start 都是新文件**：文件名 `<prefix>_<时间戳>_pNNN.rusrec`（本次运行第一个文件无
  `_pNNN`），因此 stop→start 不会截断 / 覆盖上一次录音；时间戳取自**节点启动时刻**，序号递增。
- **停录期间**：订阅仍在收，但消息不入队（静默丢弃，不计入 `dropped` —— 那是异常丢数据的计数）；
  再次 `recorder_start` 从当前时刻继续，中间空档不补。
- **写失败（磁盘满等）**：节点熔断（停录 + ERROR 日志），`state=2`；已写内容仍可用
  `inspect --scan` 抢救，之后需重启节点（`ready_` 不复位）。

---

## 8. 容量估算（实测值）

| 通道 | 单条 payload | 频率 | 速率 | 1 小时 |
|------|--------------|------|------|--------|
| 0 `RobotState` | ≈ 364 B | 125 Hz（全量，默认） | ≈ 45 KB/s | ≈ 160 MiB |
| 0 `RobotState` | ≈ 364 B | 50 Hz（`state_max_rate_hz: 50`） | ≈ 18 KB/s | ≈ 65 MiB |
| 1 `SensorFrame` | ≈ 86.6 KiB（12 221 点，zstd） | 5 Hz | ≈ 433 KiB/s | ≈ 1.5 GiB |
| 1 `SensorFrame` | ≈ 2 MiB（全分辨率 ≈ 30 万点，zstd） | 5 Hz | ≈ 10 MiB/s | ≈ 36 GiB |

结论：`rs_point_stride=1` 且 `/sensor` 全分辨率时是**大吞吐**场景，
必须配合 `max_file_size_mb` 滚动（默认 512 MiB）与磁盘空间检查；
只复盘机械臂轨迹时把 `record_sensor` 关掉即可（速率降到 KB/s 量级）。

---

## 9. 版本演进约定

- `version` 只在**破坏性变更**时 +1（字段顺序 / 语义变化）；读取方遇到不支持的版本直接拒绝。
- **兼容性扩展**（推荐做法）：把新字段追加到结构末尾 + 增大 `header_size` /
  `rec_header_size` / `index_entry_size`；老读取方按尺寸跳过，新读取方按尺寸读取。
- 新增字符串 / 字段时同样追加到末尾，绝不插在中间。
