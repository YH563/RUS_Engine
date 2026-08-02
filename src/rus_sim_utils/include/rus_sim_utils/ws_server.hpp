#pragma once

#include <functional>
#include <string>
#include <thread>
#include <atomic>
#include <vector>
#include <mutex>

#include <libwebsockets.h>

namespace RusUtils {

    /**
    * @brief 轻量 WebSocket 服务器
    *
    * - 状态推送：DriverNode 调用 Broadcast() 推送 JSON 给所有客户端
    * - 指令接收：客户端发 JSON → 调用 dispatch 回调 → 回复执行结果
    */
    class WsServer {
    public:
        /// dispatch(cmd, args, result) → success
        using DispatchFn = std::function<bool(
            const std::string& cmd,
            const std::vector<double>& args,
            std::vector<double>& result)>;

        /// 日志回调(level, msg)：level 0=info, 1=warn, 2=error
        using LogFn = std::function<void(int level, const std::string& msg)>;

        WsServer() = default;
        ~WsServer();

        /**
        * @brief 启动服务器（非阻塞）
        * @param port     监听端口，若被占用自动尝试 port+1 / port+2
        * @param dispatch 指令分发回调
        * @param log      日志回调（如传入 RCLCPP 包装）
        */
        void Start(int port = 8765, DispatchFn dispatch = nullptr, LogFn log = nullptr);

        /**
        * @brief 停止服务器
        */
        void Stop();

        /**
        * @brief 向所有连接的客户端广播 JSON 字符串
        */
        void Broadcast(const std::string& json);

        /**
        * @brief 服务器是否正在运行
        */
        bool IsRunning() const { return running_; }

        // libwebsockets 协议回调（需公开给 C 回调）
        static int ws_callback(lws* wsi, lws_callback_reasons reason,
                               void* user, void* in, size_t len);

    private:
        int port_{8765};
        std::atomic<bool> running_{false};
        std::thread thread_;
        lws_context* context_{nullptr};
        DispatchFn dispatch_;
        LogFn log_;

        // 广播状态（单状态覆盖：新数据直接替换旧数据）
        std::mutex state_mutex_;
        std::string last_state_json_;
    };

}  // namespace RusDriverNode
