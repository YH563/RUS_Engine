# RUS_Sim 前端通信客户端设计（C#）

> 本文档只描述 **前端通信客户端**（连 bridge 的那一层），不含 UI。
> 协议契约以 [ws_protocol.md](./ws_protocol.md)（v0.2）为准，客户端负责：
>
> 1. 建立三个 WebSocket 连接（`/control` `/state` `/sensor`）；
> 2. 指令下发 + 回执匹配（reply 按 id，event 按 ack_id）；
> 3. 状态流 / 感知流接收；
> 4. 断线重连。

---

## 1. 类总览

```
┌─────────────────────────────────────────────────────┐
│ BridgeClient（对外门面）                              │
│   客户端库唯一的公共入口，供上层（UI/VM）调用            │
└───────────┬──────────────────────────┬───────────────┘
            │ 调用                      │ 消息到达
┌───────────▼───────────┐   ┌──────────▼──────────────┐
│ ConnectionManager     │   │ BridgeProtocol（静态）    │
│ WebSocket 连接/收发/重连│   │ 消息模型 + JSON 编解码     │
└───────────┬───────────┘   └─────────────────────────┘
            │ 原始字节
            ▼
   WebSocket 传输（三通道）
```

三个类，职责单一，无多余抽象：

| 类 | 职责 | 依赖 |
|----|------|------|
| `BridgeClient` | 对外 API：指令下发、事件订阅、状态获取；内部做请求追踪 | ConnectionManager, BridgeProtocol |
| `ConnectionManager` | 连接建立 / 收发循环 / 断线重连 / 退避 | 无（用 `ClientWebSocket`） |
| `BridgeProtocol` | 消息模型定义 + JSON 编解码（纯函数） | 无 |

---

## 2. 消息模型（BridgeProtocol.cs）

对应协议 §2 / §3。reply 与 event 同构，用一个类。

```csharp
public static class BridgeProtocol
{
    // ---- 请求（前端 → 后端）----
    public sealed record Command(uint Id, string Cmd, double[] Args);

    // ---- 回执 / 事件（后端 → 前端，同构）----
    public sealed record ReplyOrEvent(
        string Type,        // "reply" / "event"
        uint Id,            // reply: 对应 command id；event: 恒 0
        uint AckId,         // 仅 event：触发它的 command id
        string Event,       // 仅 event：事件名
        bool Success,
        string Message,
        double[] Result);

    // ---- 状态帧（/state 通道）----
    public sealed record StateFrame(
        double Timestamp, double FrameRate,
        double[] JointPos, double[] JointVel, double[] JointAcc,
        double[] Effort, double[] FlangePos);
}
```

编解码（静态方法）：

```csharp
    public static string Encode(Command cmd);                          // 序列化 command
    public static ReplyOrEvent? TryParseReply(string json);            // 解析 reply / event
    public static StateFrame? TryParseState(string json);              // 解析 state 帧
```

> 各连接收到的 JSON 先经 `TryParseReply` / `TryParseState` 分流；解析失败直接丢弃并记录日志。
> `/sensor` 二进制帧（`uint32 LE 头长 + JSON 头 + payload`）预留，后续在 BridgeProtocol 增加解码方法，不影响现有结构。

常量放 `ProtocolConstants.cs`：

```csharp
public static class Channels  { public const string Control = "/control"; public const string State = "/state"; public const string Sensor = "/sensor"; }
public static class Commands  { public const string MoveJ = "movej"; /* 与 command_defs.hpp 对齐 */ }
public static class Events    { public const string PlanDone = "plan_done"; /* 与 §5 事件清单对齐 */ }
```

---

## 3. 对外门面：BridgeClient

```csharp
public sealed class BridgeClient : IDisposable
{
    /// <summary>host 默认 127.0.0.1，port 默认 8765</summary>
    public BridgeClient(string host = "127.0.0.1", ushort port = 8765);

    // ---- 连接 ----
    public bool IsConnected { get; }              // /control 是否在线
    public event Action<bool>? ConnectionChanged; // 连接/断线通知

    public Task ConnectAsync();                   // 连 /control（必连），断线自动重连
    public void StartStateStream();               // 需要状态可视化时调用，连 /state
    public void StopStateStream();

    // ---- 指令（阻塞式，等 reply 返回）----
    public Task<CommandResult> SendAsync(string cmd, double[]? args = null,
        int timeoutMs = 5000, CancellationToken ct = default);

    // ---- 异步事件（长任务完成通知，如 plan_done）----
    public event Action<EventNotification>? EventReceived;

    // ---- 状态流（只保留最新一帧）----
    public StateFrame? LatestState { get; }
    public event Action<StateFrame>? StateUpdated;
}
```

对上层友好的返回值封装（屏蔽协议原始字段）：

```csharp
public sealed record CommandResult(bool Success, string Message, double[] Result);
public sealed record EventNotification(string EventName, bool Success, string Message);
```

**内部实现要点**（都藏在 BridgeClient 里，不拆类）：

- 指令下发：`SendAsync` 内部 id 自增 → 序列化 → 发给 `/control` → 以
  `ConcurrentDictionary<uint, TaskCompletionSource<CommandResult>>` 挂起等待；
  reply 按 id 匹配并完成对应 TCS。
- 长任务闭环：指令先收 reply（成功=已受理），之后收到 `event` 时查
  ack_id 关联的请求并广播 `EventReceived`。上层若要"指令 → 完成"全链路，
  可在 SendAsync 之后订阅对应事件名。
- 超时：`timeoutMs`（默认 5000）内未收到 reply → 返回 `Success=false, Message="timeout"`，
  并从字典移除。
- 断线：`ConnectionChanged` 通知上层，所有未决请求全部置为失败。

---

## 4. 连接管理：ConnectionManager

```csharp
internal sealed class ConnectionManager
{
    public ConnectionManager(string host, ushort port);

    public bool IsControlConnected { get; }
    public event Action<ReadOnlyMemory<byte>>? ControlMessageReceived;  // /control 通道
    public event Action<ReadOnlyMemory<byte>>? StateMessageReceived;    // /state 通道
    public event Action? ControlDisconnected;

    public Task ConnectControlAsync(CancellationToken ct);
    public Task ConnectStateAsync(CancellationToken ct);
    public Task SendAsync(ReadOnlyMemory<byte> data, CancellationToken ct);  // 走 /control
    public void DisconnectAll();
}
```

设计要点：

- **每个通道一个 `ClientWebSocket` + 一个接收循环**。文本消息整帧接收后
  触发对应 `MessageReceived`；二进制（/sensor，预留）原样转发给上层解码。
- **重连退避**：`/control` 断线后按 500ms → 1s → 2s → 5s → 10s（封顶）重试，
  连上即重置；`/state` 断线后由 BridgeClient 的 `StartStateStream` 再次触发连接。
- 一个收一个发，不共用一个循环，避免大 payload 阻塞指令。

---

## 5. 客户端状态机

```
      ┌─────────┐   ConnectAsync    ┌──────────┐   WS 连上   ┌─────────┐
      │ Created │ ────────────────→ │ Connecting │ ─────────→ │Connected│
      └─────────┘                   └──────────┘             └────┬────┘
                                                                  │ 断线
                                               ┌──────────────────┘
                                               ▼
                                          ┌──────────┐   自动重连   ┌─────────┐
                                          │Disconnected│ ─────────→ │Connecting│
                                          └──────────┘               └─────────┘
```

- `Connected`：可发指令、可收数据。
- `Disconnected`：未决请求全部失败，`ConnectionChanged(false)` 通知上层；重连循环自动进行。

---

## 6. 时序示例

**同步指令（get_state）：**

```
UI/Sender         BridgeClient              ConnectionManager        bridge
   │ SendAsync("get_state")                     │                      │
   │ ──────────────→   id=5 挂起 pending         │  Encode → Send       │
   │                    ───────────────────────→│────────────────────→ │
   │                    │                       │                      │
   │                    │←──── JSON ────────────│←─────────── reply(id=5)
   │                    │ 按 id=5 匹配，完成 TCS │                      │
   │ ←── CommandResult ──┘                      │                      │
```

**长任务（plan → plan_done）：**

```
   │ SendAsync("plan")                          │                      │
   │ ──────→  id=6 挂起                         │── command(id=6) ───→│
   │          │←── reply(id=6, success=true) ←──│←────────────────────│ 受理
   │ ←── CommandResult(success) ──┘             │                      │
   │          │                                 │                      │
   │          │←── event(ack_id=6, plan_done) ←─│←────── module_events │ 完成
   │ ←── EventReceived(plan_done) ──┘           │                      │
```

---

## 7. 生命周期与使用方式

```csharp
// 应用启动时
var bridge = new BridgeClient();
bridge.EventReceived += OnEvent;        // 订阅异步事件
bridge.StateUpdated += OnStateUpdated;  // 订阅状态流
await bridge.ConnectAsync();            // 连 /control，自动重连
bridge.StartStateStream();              // 需要状态时开启 /state

// 需要状态可视化时开启 /state
// 应用退出时
bridge.Dispose();                       // 断开所有连接，完成未决请求
```

---

## 8. 测试要点

| 对象 | 方式 |
|------|------|
| BridgeProtocol | 用 ws_protocol.md 样例 JSON 做单元测试（reply / event / state 解析） |
| BridgeClient.SendAsync | 注入 fake ConnectionManager，验证 id 自增 / reply 匹配 / 超时返回失败 |
| 断线重连 | fake 连接模拟断线，验证退避序列与未决请求置失败 |
| 集成 | 连本地 bridge，实发 `get_state` / `stop`，比对协议文档 |
