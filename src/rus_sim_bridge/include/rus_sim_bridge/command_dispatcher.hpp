#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rus_sim_interfaces/srv/command_service.hpp>

#include <rus_sim_utils/command_registry.hpp>
#include <rus_sim_utils/command_types.hpp>
#include <rus_sim_utils/protocol.hpp>

#include "rus_sim_bridge/ws_server.hpp"

namespace rus_sim_bridge {

    /**
     * @brief 指令下发器：解析指令 → 按注册表路由 → 服务扇出 → 回执聚合
     *
     * 从 BridgeNode 剥离的职责，让桥接节点专注 WS 网关与状态/事件转发。
     *
     * - 持有 CommandRegistry（指令名 → 目标模块），启动时初始化
     * - 持有下游服务客户端（按服务名惰性创建）
     * - 支持：单模块下发 / 多模块扇出 / 本地处理（空模块列表）/ 超时检测
     *
     * 线程模型：所有方法仅在执行器线程调用（与 BridgeNode 一致）。
     */
    class CommandDispatcher {
    public:
        using CommandService = rus_sim_interfaces::srv::CommandService;

        /**
         * @param node 所属节点（shared_ptr；节点生命周期长于本对象）
         */
        explicit CommandDispatcher(std::shared_ptr<rclcpp::Node> node);

        /**
         * @brief 对外分发入口：解析 → 路由 → 下发
         * @param cmd   前端指令（已通过 ParseCommandMessage）
         * @param reply 回执回调（异步：扇出全部返回 / 失败 / 超时后调用）
         */
        void Dispatch(const RusUtils::CommandMessage& cmd, WsServer::ReplyFn reply);

        /// 下游服务无响应超时检测（由节点定时器周期调用）
        void CheckTimeouts();

    private:
        /// 一次扇出调用的共享上下文
        struct FanOutContext {
            WsServer::ReplyFn reply;
            uint32_t request_id = 0;
            size_t pending = 0;
            bool done = false;
            bool all_success = true;
            std::string message;   // 首个失败信息（空 = 全部成功）
            std::vector<double> result;
            double deadline = 0.0;
        };

        void init_routing();
        void handle_local(const RusUtils::CommandMessage& cmd, const WsServer::ReplyFn& reply);
        void call_downstream(RusUtils::Module module, const RusUtils::CommandMessage& cmd,
                             const std::shared_ptr<FanOutContext>& ctx);
        void finish_fanout(const std::shared_ptr<FanOutContext>& ctx, bool success,
                           const std::string& message, std::vector<double> result);

        std::shared_ptr<rclcpp::Node> node_;
        int timeout_ms_ = 5000;

        // 指令 → 目标模块注册表
        RusUtils::CommandRegistry registry_;

        // 服务客户端（按服务名惰性创建）
        std::map<std::string, rclcpp::Client<CommandService>::SharedPtr> clients_;

        // 进行中的扇出上下文
        std::vector<std::weak_ptr<FanOutContext>> active_;
    };

}  // namespace rus_sim_bridge
