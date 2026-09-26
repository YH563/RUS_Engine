# RUS_Sim 前端通信协议 v0.4

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
> 5. **模式切换**：`set_mode` 在**手动（直控 driver）/ 自动（planning 协调）**之间切换，
>    影响 `pause` / `resume` / `reset` / `query_motion_done` / `stop` 的路由（见 §4.1 / §4.6）。
> 6. **单位统一**：运动指令参数在协议层统一使用 **位置 m + 角度 rad**（速度参数为比例 / 百分比），
>    单位换算（m↔mm、rad↔deg）**只在驱动内部发生**，前端不做任何换算；完整量纲见 §4 参数量纲表。
> 7. **点动上限后端化**：`start_jog` 的单次位移上限改由后端配置（`driver_params.yaml` 的
>    `jog_max_dis_joint` / `jog_max_dis_trans` / `jog_max_dis_rot`）决定；第 6 个参数 `max_dis`
>    仍可传但不生效（协议兼容保留），前端**不必再传**。
> 8. **回放指令 / 文本结果（v0.4 新增）**：`replay_*` 由回放节点（`/replayer/command`）处理，
>    用于离线复盘录音（§4.7）；回执/事件新增可选字段 **`strings`（string[]）** 承载文本结果
>    （数值仍走 `result`；两者可同时出现，见 §3.1）。
> 9. **录制开关指令（v0.4 新增）**：`recorder_start` / `recorder_stop` / `recorder_status` 由录制节点
>    （`/recorder/command`）处理，用于运行期开 / 关落盘（§4.8）。节点默认**启动即录**
>    （参数 `autostart=true`），因此不接这三条指令时行为与以前完全一致。

---

## 1. 通道（三个 WebSocket 连接）

同一端口（默认 8765），前端按路径建立连接：

| 连接路径 | 承载内容 | 可靠性 | 用途 |
|----------|----------|--------|------|
| `/control` | `command` / `reply` / `event` | 可靠（id 关联回执） | 指令收发、事件通知 |
| `/state` | `state`（关节状态高频流） | 可丢帧（只发最新值） | 状态可视化 |
| `/sensor` | 感知二进制帧（点云已接通） | 可丢帧（只发最新一帧） | 点云 / 图像 |

前端至少连 `/control`。`/state`、`/sensor` 按需连接；**未连接的通道不会收到数据**。

---

## 2. 通路 A：前端 → 后端（command）

前端所有主动操作（查询 / 下发）统一为一条 `command` 消息。

```json
{ "id": 1, "cmd": "is_motion_done", "args": [] }
{ "id": 2, "cmd": "movej", "args": [0.1, 0.2, 0.3, 0, 0, 0, 0.5] }
{ "id": 3, "cmd": "set_mode", "args": [1] }
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
- **运动类指令的单位约定**（`movej` / `movel` / `servo_*` / `start_jog`）见 §4 开头的量纲说明：
  位置 m、角度 rad；`movej`/`movel` 的 `speed`/`acc` 为**比例 0~1**，`start_jog` 的 `speed%`/`acc%` 为**百分比 0~100**。
  如上例 `movej` 的 `args` = `[q1..q6(rad), speed(比例)]`，即 `0.5` 表示 50% 速度。

---

## 3. 通路 B：后端 → 前端

### 3.1 统一回执：reply / event（同构）

两者字段一致，仅语义不同：

- `reply` —— 对 command 的**同步**应答（查询结果 / 校验失败 / 下发结果）
- `event` —— 子模块的**异步**通知（长任务完成 / 错误），由 bridge 订阅 `/module_events`
  topic 转发而来

```json
// reply：成功 / 失败
{ "type": "reply", "id": 1, "success": true,  "message": "ok", "result": [1.0], "strings": [] }
{ "type": "reply", "id": 2, "success": false, "message": "unknown command: xxx", "result": [], "strings": [] }

// reply：查询类带文本结果（v0.4；例：replay_list 列出录音文件）
{ "type": "reply", "id": 3, "success": true, "message": "4 个录音文件（清单见 strings）",
  "result": [4.0, 0.0], "strings": ["run_20260926_120000.rusrec", "run_20260926_120500.rusrec"] }

// event：异步完成通知（id 固定 0，ack_id 关联原指令）
{ "type": "event", "id": 0, "ack_id": 3, "event": "pre_scan_done",
  "success": true, "message": "", "result": [], "strings": [] }
{ "type": "event", "id": 0, "ack_id": 3, "event": "error",
  "success": false, "message": "plan failed: 未完成预扫查（无点云数据）", "result": [], "strings": [] }
```

| 字段 | 类型 | reply | event | 说明 |
|------|------|-------|-------|------|
| `type` | string | ✅ | ✅ | `reply` / `event`（按此分支解析） |
| `id` | uint32 | ✅ | 固定 `0` | reply 关联 command id；event 恒为 0 |
| `ack_id` | uint32 | ❌ | ✅ | 触发该事件的 command id（0 = 模块自发，无关联） |
| `event` | string | ❌ | ✅ | 事件名（见 §5） |
| `success` | bool | ✅ | ✅ | 是否成功 |
| `message` | string | ✅ | ✅ | 错误描述 / 附加说明（成功可为空串；多模块扇出成功时统一为 `ok`） |
| `result` | double[] | ✅ | ✅ | **数值**结果（查询类带数据，操作类为空数组） |
| `strings` | string[] | ✅ | ✅ | **文本**结果（v0.4；查询类带文本，如录音文件清单 / 当前文件名）；旧前端可忽略该字段 |

> **`result` vs `strings`**：两者都是"查询类指令的结果载荷"，只是类型不同（协议层任何参数/回执
> 的数值通道只有 `double`，文本必须走 `strings`）。操作类指令两者都为空数组；失败时
> **一律看 `message`**（`result` / `strings` 可能为空）。

**路由语义（重要）**：

- `reply` 只回发给**发起该指令的那条 `/control` 连接**。
- `event` 广播给**所有已连接的 `/control` 连接**（无论谁触发的）。

**长任务闭环**：前端发 `command(id=3, cmd="pre_scan_start")` →
后端立即回 `reply(id=3, success=true)` → 子模块完成后 bridge 推
`event(ack_id=…, event="pre_scan_done", ...)`。前端可用 ack_id 把事件挂回原请求。

> **ack_id 的实际归属**（以当前后端实现为准）：
> - `pre_scan_done` / `error`（预扫查门失败）挂在 **`pre_scan_end`** 的 id 上
>   （预扫查的"完成判定"发生在 `pre_scan_end` 指令处理时）；
> - `plan_done` 挂在 `plan` 的 id 上；
> - `scan_done` 挂在 `execute` 的 id 上；
> - `error`（plan 阶段）挂在 `plan` 的 id 上。

**超时**：指令转发到子模块后，默认 **5000ms** 无响应 → bridge 回
`reply(success=false, message="timeout")`。

### 3.2 状态流：state（/state 通道）

```json
{ "type": "state", "timestamp": 1234.5, "frame_rate": 125.0,
  "joint_pos": [...], "joint_vel": [...], "joint_acc": [...],
  "effort": [...], "flange_pos": [...],
  "tool_index": 0, "tool_pose": [...] }
```

| 字段 | 类型 | 说明 |
|------|------|------|
| `type` | string | 固定 `state` |
| `timestamp` | double | 驱动侧仿真时间戳（秒） |
| `frame_rate` | double | 帧率（bridge 按相邻两帧时间差计算） |
| `joint_pos` / `joint_vel` / `joint_acc` / `effort` | double[] | 6 维关节数组（`joint_pos` 为 **rad**，与 §4 指令里的关节角同单位，可直接回填） |
| `flange_pos` | double[] | 法兰位姿（平移 + 旋转，长度 6，m/rad） |
| `tool_index` | int | 当前工具坐标系索引（0~14，0 表示法兰坐标系） |
| `tool_pose` | double[] | 当前 TCP 位姿（基坐标系下，长度 6，m/rad） |

- 覆盖式推送：bridge 只保留**最新一帧**，慢客户端丢帧，前端须容忍帧不连续。
- **推送节奏**：每收到一次 `/driver/state` 发布即推一条（1:1，同一版本不会重复推送）；
  新连接会立刻收到最近一条。
- 数组长度固定为 6（关节 1~6）。

### 3.3 感知流：sensor（/sensor 通道，点云已接通）

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
// 点云（zstd 压缩；x/y/z 是 int16 量化值，必须用 range_min/max 反量化）
{ "type": "pointcloud", "points": 307200, "fields": ["x","y","z","rgb"], "dtype": "int16",
  "range_min": [-0.421336, -0.203074, 0.104759], "range_max": [0.593334, 0.451876, 1.900208],
  "frame_id": "base_link", "encoding": "zstd", "scope": "frame",
  "timestamp": 1234.5, "seq": 1024 }
// 图像
{ "type": "image", "width": 1920, "height": 1080, "image_encoding": "rgb8", "step": 5760,
  "frame_id": "camera_color_optical_frame", "encoding": "raw", "scope": "frame",
  "timestamp": 1234.6, "seq": 1024 }
```

| 头字段 | 类型 | 适用 | 说明 |
|--------|------|------|------|
| `type` | string | 全部 | `pointcloud` / `image` / `ultrasound`（预留）/ `compressed`（预留） |
| `timestamp` | double | 全部 | 采集时间戳（秒，ROS 时基） |
| `seq` | uint32 | 全部 | 帧序号（前端检测丢帧；每类型独立递增） |
| `frame_id` | string | 全部 | 数据坐标系；点云为 `base_link`（已算好变换，前端不再做坐标变换） |
| `encoding` | string | 全部 | **payload 压缩算法**：`zstd` / `raw`（前端按此解压，不要假设） |
| `scope` | string | 全部 | 数据语义：`frame`（单视角当前帧）/ `map`（累积地图快照）/ `""`（未知） |
| `points` | uint32 | 点云 | 点数 |
| `fields` | string[] | 点云 | 分量顺序（当前固定 `x,y,z,rgb`） |
| `dtype` | string | 点云 | `x/y/z` 的类型（当前 `int16`）；`rgb` 为 uint32 0x00RRGGBB |
| `range_min` / `range_max` | double[3] | 点云 | int16 量化包围盒；反量化 `v = min + (q + 32768) * (max - min) / 65535` |
| `width` / `height` | uint32 | 图像 | 宽高（像素） |
| `image_encoding` | string | 图像 | **压缩前**像素格式：rgb8 / bgr8 / mono8 ... |
| `step` | uint32 | 图像 | 每行字节数（含 padding） |

> - **`encoding` vs `image_encoding`**：前者是压缩算法、后者是像素格式，与 ROS 侧
>   `SensorFrame.msg` 同名同义，不要混用。
> - **点云 payload 解压后布局**（每点 10 字节，`count = points`）：
>   `int16 x | int16 y | int16 z | uint32 rgb`，小端；坐标须按上面的公式反量化。
> - **链路现状（已接通）**：`rus_sim_perception` 按本格式发布 ROS 话题 `/sensor/pointcloud`
>   （`SensorFrame.msg`，与 `/preprocessed_cloud` 同频，QoS `transient_local`）；
>   `rus_sim_bridge` 已订阅该话题（`bridge_params.yaml::sensor_topic`，
>   `forward_sensor: false` 可关闭）并转成上述线格式广播到 `/sensor` 通道，
>   **一帧 = 一条 WS 二进制消息**（`LWS_WRITE_BINARY`）。
> - **完整性自检**：`4 + headLen + payloadLen == 消息长度`。兆级帧会被 TCP/WS 分片，
>   客户端库会拼回**一条消息**，但**绝不能**按「收到一次数据 = 一帧」处理。
> - **首帧延迟**：连接成功后 bridge 立即推送缓存的最新一帧，无需等下一次发布。
> - **丢帧语义**：覆盖式——bridge 只留最新一帧，慢客户端丢帧而非积压；`seq` 跳跃即丢帧。
>   ⚠️ `map_clear` 会把 `seq` 复位为 0，丢帧统计须容忍序号回退（别用 `seq` 单调递增做断言）。
> - **码率提示**：`scope=frame`（`mapping_mode=none`）10 Hz 单帧，`scope=map`
>   （`rolling` / `accumulate`，默认）是**全量地图快照**、0.5 Hz，单帧可达数 MB。
> - **`scope` 的语义**：同一话题在 `mapping_mode=none` 下是当前帧、在 `rolling` /
>   `accumulate` 下是地图快照（点数远大于单帧）。两者**都是自包含的完整点集**——`map`
>   是 `MapManager::Snapshot()` 的整图拷贝，逐帧重新量化 + 独立 zstd 压缩，**协议里没有
>   delta / 差分字段**，所以前端**一律整帧替换**，不存在「增量拼接」这种写法。
> - **`scope` 的唯一用途**：决定这帧归属哪个视图（当前帧 / 累积地图）以及点数预期
>   （是否需要降采样渲染）。**当前阶段前端只按 `scope=frame` 实现**：收到 `scope=map`
>   时走同一条「整帧替换」路径兜底即可（不崩、不 merge），不做快照抽帧 / 降采样调度。
>   ⚠️ 注意 `mapping_mode=none` 会让 `/preprocessed_cloud`（planning 输入）**一起**退化为
>   单视角——`perception` 的 `publish_cloud()` 是 raw + 压缩帧的**同一出口**，没有独立开关，
>   故「前端只想看单帧」与「planning 要累积地图」在当前实现下互斥（要两者兼得须改代码，
>   不是改配置）。

#### 前端解码参考（C#）

> 依赖：`System.Text.Json`（.NET 内置）+ `ZstdSharp.Port`（.NET 没有内置 zstd：
> `dotnet add package ZstdSharp.Port`）。

**① 接收：一条 WS 消息 = 一帧**

```csharp
using System.Net.WebSockets;

using var ws = new ClientWebSocket();
await ws.ConnectAsync(new Uri("ws://127.0.0.1:8765/sensor"), ct);

var chunk = new byte[64 * 1024];        // 分片缓冲：大小不影响帧长，只影响分片次数
using var acc = new MemoryStream();     // 复用，避免每帧新建

while (ws.State == WebSocketState.Open)
{
    acc.SetLength(0);
    WebSocketReceiveResult r;
    do                                  // ← 必须循环到 EndOfMessage：兆级帧一定会分片
    {
        r = await ws.ReceiveAsync(new ArraySegment<byte>(chunk), ct);
        if (r.MessageType == WebSocketMessageType.Close)
        {
            await ws.CloseOutputAsync(WebSocketCloseStatus.NormalClosure, null, ct);
            return;
        }
        acc.Write(chunk, 0, r.Count);
    } while (!r.EndOfMessage);

    byte[] frame = acc.ToArray();       // 到这里才是「一条完整帧」
    // 解析放后台线程（见 ②），UI 线程只消费最新一帧
    var cloud = SensorFrameDecoder.Decode(frame);
    if (cloud is not null) Dispatcher.UIThread.Post(() => Renderer.Update(cloud));
}
```

**② 解码：`uint32 LE 头长 + JSON 头 + payload`**

```csharp
using System.Buffers.Binary;
using System.Text;
using System.Text.Json;
using ZstdSharp;

public sealed record PointCloud(float[] Xyz, uint[] Rgb, int Count, string Scope, uint Seq);

public static class SensorFrameDecoder
{
    public static PointCloud? Decode(byte[] msg)
    {
        // ① 头长度（小端 uint32）
        if (msg.Length < 4) return null;
        int headLen = (int)BinaryPrimitives.ReadUInt32LittleEndian(msg);
        if (headLen <= 0 || 4 + headLen > msg.Length) return null;   // 帧不自洽 → 整帧丢弃

        // ② JSON 头（UTF-8）
        using var doc = JsonDocument.Parse(Encoding.UTF8.GetString(msg, 4, headLen));
        var h = doc.RootElement;
        if (h.GetProperty("type").GetString() != "pointcloud") return null;   // 按 type 分流

        uint   points = h.GetProperty("points").GetUInt32();
        uint   seq    = h.GetProperty("seq").GetUInt32();
        string enc    = h.GetProperty("encoding").GetString()!;  // "zstd" / "raw"，不要假设
        string scope  = h.GetProperty("scope").GetString() ?? "";
        double[] mn   = ReadArr(h, "range_min");                 // 逐帧变化，禁止缓存
        double[] mx   = ReadArr(h, "range_max");

        // ③ payload（必须复制出来：msg 缓冲会被下一帧覆盖）
        int payloadLen = msg.Length - 4 - headLen;
        if (payloadLen <= 0) return null;
        byte[] payload = msg.AsSpan(4 + headLen, payloadLen).ToArray();

        // ④ 解压（按 encoding 分支，不要写死 zstd）
        byte[] raw = enc switch
        {
            "zstd" => new Decompressor().Unwrap(payload),
            "raw"  => payload,
            _      => throw new NotSupportedException($"unknown encoding={enc}")
        };

        // ⑤ 自检：解压后必须正好 points × 10 字节（每点 int16 x/y/z + uint32 rgb）
        if (raw.Length != points * 10)
            throw new InvalidDataException($"raw={raw.Length} != points*10={points * 10}");

        // ⑥ 反量化 + 取色
        var xyz = new float[points * 3];
        var rgb = new uint[points];
        for (int i = 0; i < points; i++)
        {
            var p = raw.AsSpan(i * 10, 10);
            short qx = BinaryPrimitives.ReadInt16LittleEndian(p);
            short qy = BinaryPrimitives.ReadInt16LittleEndian(p[2..]);
            short qz = BinaryPrimitives.ReadInt16LittleEndian(p[4..]);
            rgb[i]   = BinaryPrimitives.ReadUInt32LittleEndian(p[6..]);   // 0x00RRGGBB

            xyz[i * 3]     = Dequant(qx, mn[0], mx[0]);
            xyz[i * 3 + 1] = Dequant(qy, mn[1], mx[1]);
            xyz[i * 3 + 2] = Dequant(qz, mn[2], mx[2]);
        }
        return new PointCloud(xyz, rgb, (int)points, scope, seq);
    }

    // 与后端 SensorEncoder::quantize 严格互逆
    private static float Dequant(short q, double min, double max) =>
        (float)(min + (q + 32768.0) * (max - min) / 65535.0);

    private static double[] ReadArr(JsonElement h, string key)
    {
        var a = h.GetProperty(key);
        var v = new double[a.GetArrayLength()];
        int i = 0;
        foreach (var e in a.EnumerateArray()) v[i++] = e.GetDouble();
        return v;
    }
}
```

**踩坑清单**

1. **必须循环收到 `EndOfMessage`**：2~4 MB 的帧必然被分片，`ClientWebSocket` 不会替你拼消息；
   按「收到一次数据 = 一帧」写必然解析失败。
2. **全小端**：统一用 `BinaryPrimitives.Read*LittleEndian`（x64/ARM 都明确）；不要用
   `BitConverter` 再假定本机端序。
3. **`encoding` 要分支**：点云当前固定 `zstd`，但 `raw` 也合法（是协议字段，不是实现细节）。
4. **`range_min/max` 逐帧变化**：只能用本帧头里的值，禁止缓存复用。`max > min` 恒成立
   （后端对退化包围盒补了 1 mm，不会除零）。
5. **精度**：JSON 头里的浮点按 `%.6f` 输出（≤5e-7 m 误差），量化步长 ≈ `(max-min)/65535`，
   故反量化总误差 ≤ 步长/2 + 5e-7 m（典型 1.5e-5 m）。
6. **颜色**：`rgb` 是 `0x00RRGGBB` 打包整数，取色
   `Color.FromRgb((byte)(c >> 16), (byte)(c >> 8), (byte)c)`；不要按 PCL 的 float 位模式解释。
7. **UI 线程别解压**：30 万点解压 + 反量化请放后台线程，渲染侧只保留最新一帧（覆盖式）；
   bridge 不会等慢客户端，帧会被丢而不是积压。
8. **`seq` 会回退**：`map_clear` 后 `seq` 复位为 0，丢帧统计/序号断言必须容忍回退。
9. **`scope` 决定渲染策略**：`frame` 是当前帧（整体替换）、`map` 是累积地图快照（点数大得多）。

---

## 4. 指令清单（按路由分组）

> 指令名、参数校验、路由目标以后端代码为准（`rus_sim_utils` + bridge 注册表）。
> 下表按 bridge 当前路由分组，仅用于前端参考。
>
> **路由分组与模式的关系**：业务指令（`pre_scan_*` / `set_*_pose` / `plan` / `execute` /
> `query_prescan_done`）**始终**路由到 PLANNING；直控指令（`movej` / `movel` / `servo_*` /
> `start_jog` / `robot_enable` 等）**始终**路由到 DRIVER。仅 `pause` / `resume` / `reset` /
> `query_motion_done` / `stop` 随模式切换（见 §4.6）。
>
> **运动参数量纲（单位约定，前端按此构造参数）**：
>
> 指令层单位**统一为「位置 m + 角度 rad」**，所有换算（m↔mm、rad↔deg、比例↔百分比）**只在驱动内部发生**，
> 前端不需要做任何单位转换；下表数值可直接由通路 B 的状态回填（§3.2），两侧物理语义一致（sim / real 相同）。
>
> | 参数 | 指令层单位 | 取值 / 缺省 | 驱动内部换算 |
> |------|-----------|------------|-------------|
> | `movej` `q1..q6` | **rad** | 6 维关节角，与 `state.joint_pos` 同单位 | 真实驱动 ×180/π → SDK ° |
> | `movel` `x,y,z` | **m** | 基坐标系下 TCP 平移 | 真实驱动 ×1000 → SDK mm |
> | `movel` `rx,ry,rz` | **rad** | TCP 姿态，固定轴 XYZ（`R = Rz·Ry·Rx`） | 真实驱动 ×180/π → SDK ° |
> | `servoj` `q1..q6` | **rad** | 下一控制周期关节目标 | 真实驱动 ×180/π → SDK ° |
> | `servo_cart` `x,y,z` / `rx,ry,rz` | **m** / **rad** | 下一控制周期 TCP 目标 | 真实驱动 ×1000 / ×180/π |
> | `movej` / `movel` 的 `speed` / `acc` | **比例 [0~1]** | 缺省 `0.5`（= 50%）；小于 `0.01` 按 `0.01` 处理 | 内部归一；真实驱动 ×100 → SDK 百分比 |
> | `start_jog` `speed%` / `acc%` | **百分比 [0~100]** | 缺省 50%（沿用内部默认比例 0.5） | 内部 /100 → 比例（真机再 ×100） |
> | `start_jog` `max_dis`（第 6 参数） | **已不生效**（协议兼容保留，可不传） | 实际限制由后端配置决定：`driver_params.yaml` 的 `jog_max_dis_joint`（**rad**，关节点动）/ `jog_max_dis_trans`（**m**，平移轴 `axis=1~3`）/ `jog_max_dis_rot`（**rad**，旋转轴 `axis=4~6`），`0` = 不限制 | 配置值在驱动内部换算：rad→° / m→mm；配置为 `0` 时用足够大的 SDK 上限近似 |
> | 状态回填（§3.2） | `joint_pos` **rad**、`flange_pos` / `tool_pose` **m + rad** | — | 真实驱动内部已换算回 m/rad |
>
> 补充说明：
> - 位姿旋转为**固定轴 XYZ 欧拉角**（`R = Rz·Ry·Rx`）；笛卡尔指令的目标是**基坐标系下的 TCP 位姿**，
>   工具（探头）坐标系变换同样由驱动内部完成，前端不需要自己换算。
> - 仿真驱动直接按指令层单位运行（无换算），因此同一组参数在 sim / real 下物理语义完全一致。
> - `start_jog` 的单次位移上限**不再由前端决定**：第 6 个参数 `max_dis` 仍会被后端解析（协议兼容，可不传），
>   但不参与运动控制；实际上限取后端配置 `jog_max_dis_joint` / `jog_max_dis_trans` / `jog_max_dis_rot`
>   （单位与指令层一致：rad / m，`0` = 不限制），前端既不需要也无法覆盖。仿真与真机同样生效。
> - `start_jog` 的上限按**一次点动行程**累计：达到上限后运动自动停住并保持当前位置（软上限，按控制
>   周期步进判定，停止位置可能与设定值有微小偏差），前端需先下发 `stop_jog_decel`（或
>   `stop_jog_immediate`）结束本次行程，之后的点动才从零重新累计。
> - 速度基准（仅供估算，前端无需换算）：`movej` 关节 1.0 rad/s、`movel` 平移 0.5 m/s / 旋转 1.0 rad/s 对应比例 1.0；
>   `start_jog` 关节 0.5 rad/s、平移 0.05 m/s、旋转 0.3 rad/s 对应 100%。
> - 伺服类（`servoj` / `servo_cart`）语义是"**下一控制周期的目标点**"：须在 `servo_start` 之后按控制周期连续下发，
>   `servo_end` 结束（planning 的 `execute` 内部按 `servo_rate_hz` = 125Hz 下发 `servo_cart`）。

### 4.1 本地指令（bridge 直接处理，不下发子模块）

| 指令名 | args | result | 说明 |
|--------|------|--------|------|
| `shutdown` | 无 | 空 | 关闭整个系统。reply 成功后再退出 |
| `set_mode` | [mode] | 空 | 模式切换：`0`=手动（直控 driver）、`1`=自动（planning 协调，默认）。影响 `pause`/`resume`/`reset`/`query_motion_done`/`stop` 的路由（见 §4.6） |

### 4.2 路由到 PLANNING（规划）

> planning 持有动作完成状态（执行中 / 暂停 / 完成），状态查询与暂停/恢复由 planning
> 内部逻辑处理；必要时由 planning 自行向 driver 下发控制（servo_end / servo_start）。
> 前端不直接问 driver，避免 driver 原始状态误导（如暂停后 driver 报完成）。
> 注意：`stop` 是急停，见 §4.6 扇出指令，直接到达 driver。
> 注意：本节的 `query_motion_done`/`pause`/`resume`/`reset` 仅在**自动模式**（`set_mode [1]`）
> 下路由到 planning；手动模式（`set_mode [0]`）下这些指令直发 driver（见 §4.3）。

| 指令名 | args | result | 说明 |
|--------|------|--------|------|
| `pre_scan_start` | 无 | 空 | 预扫查开始（完成后有 `pre_scan_done` 事件） |
| `pre_scan_end` | 无 | 空 | 预扫查结束（无点云数据则失败 + `error` 事件） |
| `set_start_pose` | [x, y, z] | 空 | 设置起点（≥3 个参数；支持 3/6/7：位置（m） / 位置（m）+RPY（rad） / 位置（m）+四元数 xyzw） |
| `set_end_pose` | [x, y, z] | 空 | 设置终点（≥3 个参数；同上） |
| `plan` | 无 | 空 | 开始规划（未完成预扫查或未设置起终点则失败 + `error` 事件；完成后有 `plan_done` 事件） |
| `execute` | 无 | 空 | 开始执行（伺服按 `servo_rate_hz` 逐点下发；完成后有 `scan_done` 事件） |
| `query_prescan_done` | 无 | [0/1] | 查询预扫查是否完成 |
| `query_motion_done` | 无 | [0/1] | 查询动作是否完成（planning 状态：执行中/暂停=0，空闲=1） |
| `pause` | 无 | 空 | 暂停扫查（发 servo_end 停伺服下发；query_motion_done 仍返回 0） |
| `resume` | 无 | 空 | 恢复被暂停的扫查（servo_start 从暂停位置继续） |
| `reset` | 无 | 空 | 复位规划状态（停伺服、清轨迹、状态归零） |

### 4.3 路由到 DRIVER（驱动）

| 指令名 | args | result | 说明 |
|--------|------|--------|------|
| `connect` | 无 | 空 | 连接机器人 |
| `disconnect` | 无 | 空 | 断开连接 |
| `is_connected` | 无 | [0/1] | 是否已连接 |
| `is_in_drag_teach` | 无 | [0/1] | 是否拖拽示教中 |
| `robot_enable` | [state] | 空 | 使能（1）/ 去使能（0） |
| `get_state` | 无 | [timestamp, q1..q6] | 获取当前关节状态 |
| `is_motion_done` | 无 | [0/1] | 运动是否完成（servo_end 后活跃伺服段已清理，正确返回 1） |
| `switch_driver` | [type, ip1, ip2, ip3, ip4] | 空 | 切换 sim(0) / real(1)，IP 四个十进制段 |
| `get_driver_type` | 无 | [0/1] | 查询当前驱动类型：0=仿真（sim），1=真实（real），与 `switch_driver` 的 type 同编码 |
| `movej` | [q1..q6, speed?, acc?] | 空 | 关节运动（≥6 个参数；`q1..q6` 单位 **rad**，`speed`/`acc` 为比例 [0~1]，缺省 0.5） |
| `movel` | [x,y,z,rx,ry,rz, speed?, acc?] | 空 | 笛卡尔直线运动（≥3 个参数：3=仅位置（姿态保持当前）、6=完整位姿；目标为 TCP 位姿，基座下 **x,y,z 为 m、rx,ry,rz 为 rad**（固定轴 XYZ）；`speed`/`acc` 为比例 [0~1]；工具坐标变换由驱动内部处理） |
| `servoj` | [q1..q6] | 空 | 关节伺服（≥6 个参数；单位 **rad**，须在 `servo_start` 后逐控制周期下发） |
| `servo_cart` | [x,y,z,rx,ry,rz] | 空 | 笛卡尔伺服（≥6 个参数；**m/rad**，基座下 TCP 位姿，须逐控制周期下发） |
| `start_jog` | [ref, axis, dir, speed%, acc%, max_dis?] | 空 | 开始点动（≥5 个参数；`axis` 1~6，`dir` 0=负/1=正，`speed%`/`acc%` 为百分比 [0~100]；第 6 个参数 `max_dis` **已不生效**（协议兼容保留，可不传），实际上限由后端 `driver_params.yaml` 决定） |
| `stop_jog_decel` | 无 | 空 | 点动减速停止 |
| `stop_jog_immediate` | 无 | 空 | 点动立即停止 |
| `servo_start` | 无 | 空 | 伺服模式开始 |
| `servo_end` | 无 | 空 | 伺服模式结束（清理活跃伺服段） |
| `run_file` | 无 | 空 | 执行指令文件（路径由后端参数配置） |
| `set_time_speed` | [speed] | 空 | 设置仿真时间倍速 |
| `get_time_speed` | 无 | [speed] | 查询倍速 |
| `get_sim_time` | 无 | [t] | 查询仿真时间 |
| `step_once` | 无 | 空 | 单步仿真 |
| `get_frame_rate` | 无 | [hz] | 查询帧率 |
| `set_tool_calib_point` | [point_num] | 空 | 六点法标定：记录第 N 个工具参考点（1~6，TCP 对准同一尖点） |
| `compute_tool_calib` | 无 | [x,y,z,rx,ry,rz] | 六点法标定：计算工具坐标系（TCP 相对法兰，m/rad，计算在驱动内部完成） |
| `set_tool_coord` | [id, x,y,z,rx,ry,rz] | 空 | 设置工具坐标系并生效（TCP 相对法兰，m/rad），同时持久化到配置文件 |
| `set_tool_index` | [id] | 空 | 切换当前工具坐标系索引（0=法兰，N=工具 N），运动参考系随之切换并持久化 |
| `get_tool_coords` | 无 | [index, count, id0(6值), ...] | 查询工具坐标系表与当前索引 |

**手动模式附加指令**（`set_mode [0]` 下 `pause`/`resume`/`reset`/`query_motion_done` 也路由到此）：

| 指令名 | args | result | 说明 |
|--------|------|--------|------|
| `pause` | 无 | 空 | 暂停运动（仅手动模式路由到此；自动模式见 §4.2） |
| `resume` | 无 | 空 | 恢复被暂停的运动（仅手动模式路由到此） |
| `reset` | [mode?, enable?] | 空 | 复位驱动：`mode>=1` 且 `enable!=0` → 完整复位重新上使能；否则软复位（仅手动模式路由到此） |
| `query_motion_done` | 无 | [0/1] | 查询运动是否完成（仅手动模式路由到此） |

### 4.4 PERCEPTION 指令

通道 / 注册已预留，指令待后续设计，暂不定义。感知大块数据（点云 / 图像）
始终走 `/sensor` 二进制通道，不通过 command 传输。

### 4.5 多目标扇出指令

多目标指令：bridge 并发转发给所有目标，**全部成功才返回 success**，
`result` 按目标顺序拼接；任一失败则 `message` 为首个失败信息。

| 指令名 | args | result | 说明 |
|--------|------|--------|------|
| `stop` | 无 | 空 | **急停**：自动模式扇出 `{PLANNING, DRIVER}`（planning 停伺服循环并更新状态，driver 立即停车）；手动模式仅发 driver（安全关键，直达） |

### 4.6 模式与路由（`set_mode` 汇总）

默认**自动**模式（`set_mode [1]`）。切换为本地指令，立即生效。

| 指令 | 自动模式（默认） | 手动模式 |
|------|------------------|----------|
| `pause` / `resume` / `reset` / `query_motion_done` | → PLANNING（planning 协调扫查） | → DRIVER（直控） |
| `stop` | 扇出 `{PLANNING, DRIVER}` | 仅 DRIVER（急停直达） |
| `pre_scan_*` / `set_*_pose` / `plan` / `execute` / `query_prescan_done` | → PLANNING | → PLANNING（不变） |
| 直控指令（`movej` / `servo_*` / `start_jog` 等） | → DRIVER | → DRIVER（不变） |
| 回放指令（`replay_*`） | → REPLAYER | → REPLAYER（不变，与手动/自动无关） |
| 录制指令（`recorder_*`） | → RECORDER | → RECORDER（不变，与手动/自动无关） |

**切手动时的联动**：bridge 在切入手动瞬间向 planning 下发一条 `stop`
（`send_raw_to_module`），终止任何进行中的扫查，避免手动直控与扫查冲突。

**建议**：前端默认使用自动模式；仅在需要底层层级调试（工程师通路）时切手动。

---

### 4.7 路由到 REPLAYER（离线回放）

> 回放节点 `replayer_node`（可执行 `rus_sim_recorder_replay`，服务 `/replayer/command`）
> 把 `.rusrec` 录音**按时间轴重发回录制时的话题**，用于无设备复现问题与前端联调；
> 录音格式与时间轴口径见 `docs/Protocol/RecFormat.md` §7.4。
>
> 与在线链路的关系：回放是**旁路** —— 默认发回录制时的话题（`/driver/state`、
> `/sensor/pointcloud`），因此 bridge 会像实时一样把它呈现到前端 `/state` / `/sensor`；
> **真驱动 / 真机在线时会撞话题**，须先停驱动或用 `topic_prefix` 隔离（见文末"回放口径"）。
> 本节 10 条指令**不随模式切换**（手动 / 自动与回放无关），始终路由到 REPLAYER。

**状态编码（`state`，出现在多个 result 的首位）**

| 值 | 含义 | 说明 |
|----|------|------|
| 0 | `idle` | 未载入 / 已 `replay_stop`（游标回到起点，文件仍载入） |
| 1 | `playing` | 正在按时间轴发布 |
| 2 | `paused` | 暂停 / 刚载入（停在起点，等 `replay_start`） |
| 3 | `finished` | 播放到末尾（`loop=true` 时不停，回到起点继续） |

**`replay_status` 的 result 字段（定长 9 项，顺序是协议的一部分）**

| 下标 | 字段 | 单位 / 取值 |
|------|------|-------------|
| 0 | `state` | 0~3（上表） |
| 1 | `progress` | s（相对文件起点；= 下一条待发布记录的时间） |
| 2 | `duration` | s（录音时长 = 末条时间 - 首条时间） |
| 3 | `speed` | 倍速（0.05 ~ 20） |
| 4 | `cursor` | 记录游标（= `records` 时表示已播完） |
| 5 | `records` | 录音记录总数（含全部通道） |
| 6 | `loaded` | 1 = 已载入录音，0 = 无 |
| 7 | `file_index` | 当前文件在目录清单中的序号 |
| 8 | `file_count` | 目录内录音文件数（含 `file_path` 指定的那一个） |

同样的 9 项也作为 `replay_start` / `replay_pause` / `replay_resume` / `replay_stop` /
`replay_seek` / `replay_step` 的 result 返回（前端无需再补一次 `replay_status`）。

| 指令名 | args | result | strings | 说明 |
|--------|------|--------|---------|------|
| `replay_list` | 无 | [文件数, 当前序号] | 全部录音文件名（文件名升序） | 列出可回放录音（前端下拉列表数据源） |
| `replay_load` | [序号?] | [文件数, 当前序号] | [已载入文件名] | 载入录音（缺省 = 保持当前序号）；载入后 `state=2`、游标归零；**无尾索引的崩溃录音同样可载入** |
| `replay_start` | [倍速?] | 9 项（见上） | — | 开始播放（未载入时按启动参数自动载入）；可选倍速（0.05~20） |
| `replay_pause` | 无 | 9 项 | — | 暂停（进度冻结；非 playing 返回失败） |
| `replay_resume` | 无 | 9 项 | — | 从暂停处继续（不跳时间） |
| `replay_stop` | 无 | 9 项 | — | 停止并复位到起点（文件保留，可直接再 start） |
| `replay_seek` | [t] | 9 项 | — | 跳到相对文件起点 `t` 秒（定位到第一条 ≥ t 的记录）；播放中跳转后以新位置重新起拍 |
| `replay_set_speed` | [speed] | [speed] | — | 设置倍速（越界钳位；播放中即时生效且不跳时间） |
| `replay_step` | [n?] | 9 项 | — | 单步发布 n 条（缺省 1；仅 paused / idle 可用） |
| `replay_status` | 无 | 9 项 | [当前文件名]（未载入时为空） | 查询状态（前端进度条 / 时间轴数据源） |

**典型时序（前端时间轴控件 → 指令映射）**

```
前端                                    后端
 │  replay_list ──────────────► reply result=[4, 0] strings=["run_…_120000.rusrec", …]
 │  replay_load [1] ──────────► reply result=[4, 1] strings=["run_…_120500.rusrec"]
 │  replay_start [2.0] ───────► reply result=[1, 0, 2.4, 2.0, 0, 30, 1, 1, 4]
 │                               （/state 与 /sensor 通道开始出现回放数据）
 │  replay_pause ─────────────► reply result=[2, 1.1, 2.4, 2.0, 13, 30, 1, 1, 4]
 │  replay_seek [0.0] ────────► reply result=[2, 0.0, 2.4, 2.0, 0, 30, 1, 1, 4]
 │  replay_resume ────────────► reply result=[1, 0.0, 2.4, 2.0, 0, 30, 1, 1, 4]
 │  ◄── event replay_done (ack_id = replay_start 的 id，result=[已发布条数])
```

**回放口径（与后端实现一致，前端不做任何换算）**

- **时间轴基准**由回放节点参数 `time_source` 决定：`stamp`（默认，消息时间戳）/ `recv`
  （录制入队时刻）；`progress`、`duration`、`replay_seek` 都在同一条轴上。
- **不重打时间戳、不改 payload**：回放原样重发录制字节，`/state.timestamp` 仍是录制时刻，
  前端解析路径与实时完全一致（只是时间在"往回走"）。
- **话题隔离**：`topic_prefix` 非空时发布到 `<prefix><原话题>`（如 `/replay/driver/state`）；
  给的是前缀而不是完整话题，故**消息类型不变**。
- **失败语义**：读盘 / CRC 校验失败 → 停止回放 + 广播 `error` 事件（`ack_id` = 触发播放的
  指令 id），**不会继续发坏数据**；载入失败（文件不存在 / 序号越界 / 无记录）由 `reply.success=false`
  + `message` 说明。
- **文件来源**：录音目录由节点参数 `record_dir`（与 recorder 的 `output_dir` 对齐）决定；
  清单只按文件名升序（recorder 文件名里含时间戳 → 即时间顺序），**不接受前端传路径**
  （协议数值通道只传 `double`，路径属于后端配置）。

### 4.8 路由到 RECORDER（录制开关）

> 录制节点 `recorder_node`（可执行 `rus_sim_recorder_node`，服务 `/recorder/command`）
> 把 `/driver/state`（通道 0）与 `/sensor/pointcloud`（通道 1）落成 `.rusrec`
> （格式与体检见 `docs/Protocol/RecFormat.md`）。本节 3 条指令用于**运行期开关落盘**：
> 不决定"录什么"（通道由启动参数 `record_state` / `record_sensor` 决定），只决定"录不录"。
>
> 默认 `autostart=true`：节点起来就开始录（等价于启动后立刻收到一次 `recorder_start`），
> 因此**不接这三条指令时行为与以前完全一致**；要"由前端决定何时开录"就设 `autostart=false`。
>
> 与在线链路的关系：录制是**旁路** —— 只订阅、不发布，各模块与前端可视化都不需要
> 感知它是否在录；`recorder_stop` 只是停写盘，话题流照常。本节 3 条指令
> **不随模式切换**（手动 / 自动与录制无关），始终路由到 RECORDER。

**状态编码（`state`，`recorder_status.result[0]`）**

| 值 | 含义 | 说明 |
|----|------|------|
| 0 | `stopped` | 未录制（未开始 / 已 `recorder_stop`）；文件保留，可直接再 `recorder_start` |
| 1 | `recording` | 录制中 |
| 2 | `failed` | 写失败熔断（磁盘满 / 无权限等）：**不可再 start**，检查磁盘后重启节点 |

**`recorder_status` 的 result 字段（定长 7 项，顺序是协议的一部分）**

| 下标 | 字段 | 单位 / 取值 |
|------|------|-------------|
| 0 | `state` | 0~2（上表） |
| 1 | `records` | 本次进程累计写入记录数（跨 `start` / `stop`，不清零） |
| 2 | `payload_mib` | 累计 payload 体积（MiB） |
| 3 | `file_mib` | 当前 / 最后文件的已写字节（MiB；未录过为 0） |
| 4 | `dropped` | 异常丢弃记录数（队列满 / 写失败）；**停录期间到达的数据不计入** |
| 5 | `throttled` | 限流丢弃记录数（`state_max_rate_hz` / `sensor_max_rate_hz` 命中） |
| 6 | `files` | 已创建的录音文件数（含当前；滚动 / 重新 `start` 都 +1） |

同样的 7 项也作为 `recorder_start` / `recorder_stop` 的 result 返回（前端无需再补一次 `recorder_status`）。

| 指令名 | args | result | strings | 说明 |
|--------|------|--------|---------|------|
| `recorder_start` | 无 | 7 项（见上） | [新文件名] | 开始录制：打开**新文件**（本次运行第一个文件 = `<prefix>_<时间戳>.rusrec`，之后一律带 `_pNNN` 后缀，**绝不覆盖**已有录音）。已在录制 / `enabled=false` / 已熔断 → `success=false` + `message` |
| `recorder_stop` | 无 | 7 项 | [已封存文件名] | 停录：先把已入队数据**写完**再封存（写尾索引 + Footer），文件可立即回放 / 体检；未在录制 → `success=false` |
| `recorder_status` | 无 | 7 项 | [当前 / 最后文件名]（从未录过时为空） | 查询状态（前端录制指示 / 计时数据源） |

**典型时序（前端录制按钮 → 指令映射）**

```
前端                                       后端
 │  recorder_status ─────────► reply result=[0, 0, 0.0, 0.0, 0, 0, 0]（未开始）
 │  recorder_start ──────────► reply result=[1, 0, 0.0, 0.0, 0, 0, 1]
 │                              strings=["run_20260926_141530.rusrec"]（新文件已建好）
 │  …（录制中：/driver/state、/sensor/pointcloud 持续入队落盘）…
 │  recorder_status ──────────► reply result=[1, 15234, 96.7, 12.4, 0, 0, 1]
 │  recorder_stop ────────────► reply result=[0, 15240, 96.8, 13.1, 0, 0, 1]
 │                              strings=["run_20260926_141530.rusrec"]（已写尾索引，可回放）
```

**录制口径（与后端实现一致，前端不做任何换算）**

- **不阻塞数据流**：订阅回调只做 CDR 序列化 + 入队，落盘由独立写线程负责；队列满
  （默认 2048 条 / 256 MiB，见 `recorder_params.yaml`）丢新并计入 `dropped`，
  **绝不拖慢 driver / perception**。
- **停录期间不计数**：`recorder_stop` 之后到达的消息**不入队、不计 `dropped`**
  （那是"异常丢数据"的计数）；再次 `recorder_start` 从**新文件 + 当前时刻**继续录，
  中间空档不补。
- **每次 start 都是新文件**：文件名 `<prefix>_<时间戳>_pNNN.rusrec`（本次运行第一个文件无
  `_pNNN`），因此 stop→start 不会截断 / 覆盖上一次录音；时间戳取自**节点启动时刻**，序号递增。
- **封存时机**：`recorder_stop` 的 reply 返回即代表尾索引已写完（random access 可用）；
  进程被强杀（未 stop）的文件没有尾索引，仍可 `inspect --scan` / 回放（`RecFormat.md` §7.4）。
- **失败语义**：`enabled=false`（不订阅不落盘）、无可用通道、输出目录不可用 →
  `recorder_start` 返回 `success=false` + 原因（节点日志有对应 ERROR / WARN）；
  写盘失败（磁盘满等）→ 节点**熔断**（停录 + ERROR 日志），此后 `state=2`、`start` 一律失败，
  需重启节点；写线程 **3 s** 内未完成开 / 封 → 该次指令返回超时失败（状态以 `recorder_status` 为准）。

## 5. 事件清单（异步通知）

| 事件名 | 触发时机 | 关联指令（ack_id） | success | result |
|--------|----------|--------------------|---------|--------|
| `pre_scan_done` | 预扫查完成（点云已就绪） | `pre_scan_end` | true | 空 |
| `plan_done` | 轨迹规划完成 | `plan` | true | 空 |
| `scan_done` | 正式扫查执行完成 / 被 stop 中断 | `execute` | true / false（中断） | 空 |
| `motion_done` | 当前运动完成（预留） | 任意运动指令 | true | 空 |
| `replay_done` | 回放播到末尾（`loop=false`；`loop=true` 时不发） | `replay_start` / `replay_resume` | true | [已发布条数] |
| `error` | 模块错误（预扫查门 / 规划失败 / 回放读盘失败等） | 触发指令 | false | 空 |

**典型失败场景**：
- 未 `pre_scan_end` 直接 `plan` → `error`，`message="plan failed: 未完成预扫查（无点云数据）"`。
- `pre_scan_end` 时无点云数据 → `error`，`message="pre_scan failed: 未收到点云数据"`。
- 未设置起终点直接 `plan` → `error`，`message="plan failed: 起终点未设置"`。

---

## 6. 完整扫查时序（自动模式示例）

```
前端                                  后端
 │  set_mode [1] ────────────────►  reply ok（默认已是自动）
 │  pre_scan_start ─────────────►  reply ok
 │                                   感知产点云 → /preprocessed_cloud
 │  pre_scan_end ───────────────►  reply ok
 │  ◄── event pre_scan_done (ack_id=pre_scan_end)
 │  set_start_pose [-0.4,-0.21,-0.16] ► reply ok
 │  set_end_pose   [-0.41,0.24,-0.17] ► reply ok
 │  plan ───────────────────────►  reply ok
 │  ◄── event plan_done (ack_id=plan)
 │  execute ────────────────────►  reply ok
 │                                   伺服逐点下发 @125Hz（约 3s）
 │  ◄── event scan_done (ack_id=execute)
 │  query_motion_done ──────────►  reply [1.0]（planning 已完成）
 │  set_mode [0] ───────────────►  reply ok（切手动，通知 planning stop）
 │  pause ──────────────────────►  reply ok（直发 driver）
 │  query_motion_done ──────────►  reply [1.0]（driver 视角）
```

---

## 7. 前端实现要点（Avalonia）

1. **三个连接**：`/control`（必连）、`/state`（状态可视化）、`/sensor`（点云已接通）。
2. **两个解析器**：
   - JSON 解析器：读 `type` 字段分流 → `reply` / `event`（同构，可复用字段绑定）/
     `state`；
   - 二进制解析器：`uint32 LE 头长 + JSON 头 + payload`（§3.3 附 C# 参考实现）。
3. **reply 与 event 同构**：建议建模为一个类（`type` / `id` / `ack_id` / `event` /
   `success` / `message` / `result`），reply 时 `event` 字段为空，event 时 `id` 为 0。
4. **请求追踪**：`id` 自增；reply 按 `id` 匹配；event 按 `ack_id` 挂回原请求。
5. **丢帧容忍**：`/state` 只保留最新值，不要依赖连续性。
6. **数值精度**：result 内所有 double 为 6 位小数字符串，解析成 double 即可。
7. **感知帧别在 UI 线程解压**：单帧解压后 2~4 MB（30 万点），放后台线程解析成点数组，
   渲染侧只取「最新一帧」（覆盖式），避免帧积压导致画面延迟。
7. **模式管理**：应用层维护当前模式（自动/手动）；切换用 `set_mode [0/1]`，
   收到 reply 后再更新 UI 上的模式状态。

---

## 8. 代码对照

| 关注点 | 头文件 |
|--------|--------|
| 通道路径 / 指令名 / 事件名 / 传感器类型常量 | `rus_sim_utils/command_defs.hpp` |
| 消息结构体 / JSON 编解码 / 传感器帧 | `rus_sim_utils/protocol.hpp` |
| 指令结构体与参数校验 | `rus_sim_utils/command_types.hpp` |
| 指令 → 模块路由配置 | `rus_sim_utils/command_registry.hpp` + bridge `command_dispatcher.cpp` |
| bridge 实际路由表 | `rus_sim_bridge/src/command_dispatcher.cpp` `init_routing()` |
| planning 状态机 / 事件发布 | `rus_sim_planning/planning_node.hpp` |
| 模块回执的文本结果通道（`strings`） | `rus_sim_interfaces/srv/CommandService.srv` + `rus_sim_utils/protocol.hpp`（`ResultMessage::strings`） |
| 回放节点（时间轴 / 泛型重发 / `replay_*` 指令） | `rus_sim_recorder/include/rus_sim_replayer/replay_node.hpp` |
| 录制节点（双路落盘 / `/recorder/command` 的 3 条 `recorder_*` 指令） | `rus_sim_recorder/include/rus_sim_recorder/recorder_node.hpp` |
| 录音文件读取（时间轴 / 随机访问） | `rus_sim_recorder/include/storage/rec_reader.hpp` |
