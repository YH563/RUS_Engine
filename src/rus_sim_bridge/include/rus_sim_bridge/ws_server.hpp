#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <libwebsockets.h>

#include <rus_sim_utils/protocol.hpp>

namespace rus_sim_bridge {

    /**
     * @brief 多通道 WebSocket 服务器（同一端口，按路径/子协议分通道）
     *
     * 通道划分：
     *   /control  —— command / reply / event（可靠，id 关联回执）
     *   /state    —— state 高频流（可丢帧，覆盖式推送最新值）
     *   /sensor   —— 感知二进制帧（可丢帧，覆盖式推送最新一帧）
     *
     * 线程模型：
     *   - 网络事件循环运行在独立线程
     *   - CommandHandler 在事件循环线程被调用（勿做阻塞操作，直接入队）
     *   - ReplyFn 线程安全，可跨线程延迟调用（用于异步路由完成后回执）
     */
    class WsServer {
    public:
        /// 回执回调：线程安全，任意时刻可调用（内部入队，事件循环推送）
        using ReplyFn = std::function<void(const std::string& json)>;

        /// 指令处理器：收到 command 后回调，自行决定何时调用 reply(json)
        using CommandHandler = std::function<void(const RusUtils::CommandMessage& cmd, ReplyFn reply)>;

        /// 日志回调(level, msg)：0=info 1=warn 2=error
        using LogFn = std::function<void(int level, const std::string& msg)>;

        WsServer() = default;
        ~WsServer();

        WsServer(const WsServer&) = delete;
        WsServer& operator=(const WsServer&) = delete;

        /**
         * @brief 启动服务器（非阻塞）
         * @param port    监听端口；被占用时自动尝试 port+1 / port+2
         * @param handler 指令处理器（事件循环线程调用，勿阻塞）
         * @param log     日志回调（可传 RCLCPP 包装）
         */
        void Start(int port = 8765, CommandHandler handler = nullptr, LogFn log = nullptr);

        /// 停止服务器并等待事件循环线程退出
        void Stop();

        bool IsRunning() const { return running_.load(); }

        /// 向所有 /state 连接推送最新状态（覆盖式，可丢帧）
        void BroadcastState(const std::string& json);

        /// 向所有 /control 连接推送事件（逐会话入队，可靠）
        void BroadcastEvent(const std::string& json);

        /**
         * @brief 向所有 /sensor 连接推送最新感知帧（覆盖式，可丢帧）
         *
         * @param frame 完整一帧的线格式字节（`uint32 LE 头长 + JSON 头 + payload`，
         *              见 RusUtils::EncodeSensorFrame）
         *
         * 语义与 /state 一致：**只保留最新一帧**，慢客户端丢帧而不是积压；
         * 一帧 = 一条 WS 二进制消息（前端按「一条消息 = 一帧」做完整性自检）。
         * 帧可到兆级，内部按需动态分配并走 LWS_WRITE_BINARY（旧的定长 16 KiB 缓冲
         * 会静默截断，勿在此通道复用 /state 的写法）。
         * 新连接的客户端会立即拿到缓存的最新一帧（无需等下一次发布）。
         */
        void BroadcastSensor(std::vector<uint8_t> frame);

        // libwebsockets 协议回调（公开给 C 回调）
        static int ws_callback(lws* wsi, lws_callback_reasons reason,
                               void* user, void* in, size_t len);

    private:
        // 内部：会话登记表 + 待推送队列（用会话 id 而非裸指针，规避 wsi 生命周期竞态）
        struct SessionInfo {
            uint64_t id = 0;
            RusUtils::Channel channel = RusUtils::Channel::Control;
            uint64_t state_sent_gen = 0;   // 本会话已推送的状态代次（0 = 尚未推送）
            uint64_t sensor_sent_gen = 0;  // 本会话已发送的感知帧代次（0 = 尚未发过）
        };

        void enqueue_session(uint64_t session_id, const std::string& json);
        void flush_scheduled();  // 事件循环线程内调用

        std::atomic<bool> running_{false};
        std::thread thread_;
        // libwebsockets C 句柄：RAII 包装，离开作用域自动 lws_context_destroy
        std::unique_ptr<lws_context, void (*)(lws_context*)> context_{nullptr, lws_context_destroy};
        int port_{8765};
        CommandHandler handler_;
        LogFn log_;

        // 状态流（覆盖式）
        std::mutex state_mutex_;
        std::string last_state_json_;
        uint64_t state_gen_ = 0;  // 每次 BroadcastState +1，会话按代次幂等推送

        // 感知流（覆盖式单槽）：shared_ptr 换出旧帧，读侧只做引用计数，
        // 避免兆级字节在锁内二次拷贝。gen 每收到一帧 +1，会话按代次判定是否需发送
        // （否则 50ms 一轮的 flush 会把同一帧反复重发）。
        std::mutex sensor_mutex_;
        std::shared_ptr<const std::vector<uint8_t>> last_sensor_frame_;
        uint64_t sensor_gen_ = 0;

        // 会话登记（wsi → 信息 / id → wsi / id → 待推送）
        mutable std::mutex registry_mutex_;
        std::unordered_map<lws*, SessionInfo> sessions_;
        std::unordered_map<uint64_t, lws*> sessions_by_id_;
        std::unordered_map<uint64_t, std::vector<std::string>> pending_replies_;
        uint64_t next_session_id_{1};
    };

}  // namespace rus_sim_bridge
