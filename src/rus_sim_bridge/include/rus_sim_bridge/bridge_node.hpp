#pragma once

#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rus_sim_interfaces/msg/robot_state.hpp>
#include <rus_sim_interfaces/srv/command_service.hpp>

#include "rus_sim_bridge/protocol.hpp"
#include "rus_sim_bridge/ws_server.hpp"

namespace rus_sim_bridge {

    /**
     * @brief 前后端桥接节点（单一入口）
     *
     * - WebSocket 多通道服务（/control /state /sensor）
     * - 统一 command 入口，按路由表转发到各模块 CommandService
     * - 高层指令可展开为多条下游指令（如 stop → planning + driver）
     * - 订阅 /driver/state 转为 StateMessage 广播到 /state 通道
     *
     * 线程模型：
     *   - WS 事件循环线程：收到 command → 入队
     *   - 执行器线程：定时器出队 → 异步调用下游服务 → 回执经 ReplyFn 返回
     */
    class BridgeNode : public rclcpp::Node {
    public:
        explicit BridgeNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    private:
        using CommandService = rus_sim_interfaces::srv::CommandService;

        /// 一条下游指令
        struct Downstream {
            std::string service;               // ROS2 服务名，如 /driver/command
            std::string command;               // 下游指令名
            std::vector<double> args;          // 固定参数；空 = 透传前端 args
        };

        /// 一次扇出调用的共享上下文
        struct FanOutContext {
            WsServer::ReplyFn reply;
            uint64_t request_id = 0;
            size_t pending = 0;
            bool done = false;
            bool all_success = true;
            std::string message;   // 首个失败信息（空 = 全部成功）
            std::vector<double> result;
            double deadline = 0.0;
        };

        void init_routing();
        void on_frontend_command(const CommandMessage& cmd, WsServer::ReplyFn reply);
        void drain_queue();   // 定时器：处理积压 command
        void route(const CommandMessage& cmd, const WsServer::ReplyFn& reply);
        void call_downstream(const Downstream& d, const std::vector<double>& frontend_args,
                             const std::shared_ptr<FanOutContext>& ctx);
        void finish_fanout(const std::shared_ptr<FanOutContext>& ctx, bool success,
                           const std::string& message, std::vector<double> result);
        void on_state(const rus_sim_interfaces::msg::RobotState::SharedPtr msg);
        void check_timeouts();  // 定时器：下游服务无响应超时

        // 指令队列（WS 线程入队 / 执行器线程出队）
        std::mutex queue_mutex_;
        std::deque<std::pair<CommandMessage, WsServer::ReplyFn>> pending_;

        // 路由表：高层指令名 → 下游指令列表；驱动透传集
        std::unordered_map<std::string, std::vector<Downstream>> high_level_;
        std::vector<std::string> driver_passthrough_;

        // 服务客户端（按服务名惰性创建，仅执行器线程访问）
        std::map<std::string, rclcpp::Client<CommandService>::SharedPtr> clients_;
        std::vector<std::weak_ptr<FanOutContext>> active_;

        rclcpp::TimerBase::SharedPtr drain_timer_;
        rclcpp::TimerBase::SharedPtr timeout_timer_;
        rclcpp::Subscription<rus_sim_interfaces::msg::RobotState>::SharedPtr state_sub_;

        WsServer ws_;
        double last_state_ts_ = -1.0;
        double state_rate_ = 0.0;
    };

}  // namespace rus_sim_bridge
