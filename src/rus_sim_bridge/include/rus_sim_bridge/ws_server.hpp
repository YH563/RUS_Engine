#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <libwebsockets.h>

#include "rus_sim_bridge/protocol.hpp"

namespace rus_sim_bridge {

    /**
     * @brief 多通道 WebSocket 服务器（同一端口，按路径/子协议分通道）
     *
     * 通道划分：
     *   /control  —— command / reply / event（可靠，id 关联回执）
     *   /state    —— state 高频流（可丢帧，覆盖式推送最新值）
     *   /sensor   —— 图像 / 点云（预留）
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
        using CommandHandler = std::function<void(const CommandMessage& cmd, ReplyFn reply)>;

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

        // libwebsockets 协议回调（公开给 C 回调）
        static int ws_callback(lws* wsi, lws_callback_reasons reason,
                               void* user, void* in, size_t len);

    private:
        // 内部：会话登记表 + 待推送队列（用会话 id 而非裸指针，规避 wsi 生命周期竞态）
        struct SessionInfo {
            uint64_t id = 0;
            Channel channel = Channel::Control;
        };

        void enqueue_session(uint64_t session_id, const std::string& json);
        void flush_scheduled();  // 事件循环线程内调用

        std::atomic<bool> running_{false};
        std::thread thread_;
        lws_context* context_{nullptr};
        int port_{8765};
        CommandHandler handler_;
        LogFn log_;

        // 状态流（覆盖式）
        std::mutex state_mutex_;
        std::string last_state_json_;

        // 会话登记（wsi → 信息 / id → wsi / id → 待推送）
        mutable std::mutex registry_mutex_;
        std::unordered_map<lws*, SessionInfo> sessions_;
        std::unordered_map<uint64_t, lws*> sessions_by_id_;
        std::unordered_map<uint64_t, std::vector<std::string>> pending_replies_;
        uint64_t next_session_id_{1};
    };

}  // namespace rus_sim_bridge
