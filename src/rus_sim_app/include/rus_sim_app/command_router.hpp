#pragma once

#include <future>
#include <memory>
#include <mutex>
#include <thread>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rus_sim_interfaces/srv/command_service.hpp"

#include "command_mapper.hpp"

namespace RusSimApp {

    /** 模块执行结果 */
    struct ModuleResult {
        bool success = false;
        std::string message;
        std::vector<double> data;
    };

    /**
     * @brief 指令路由器：统一经 CommandService 服务向各模块转发指令，异步等待结果
     *
     * 所有下游模块统一暴露 CommandService 服务（command + args → 结果）。
     * 路由器持有节点指针创建客户端，路由选择 + 异步等待由本组件完成。
     */
    class CommandRouter {
    public:
        /**
         * @param node 协调器节点（用于创建服务客户端）
         */
        explicit CommandRouter(rclcpp::Node& node);

        /**
         * @brief 注册模块通道（服务名）
         *
         * @param module       模块类型
         * @param service_name 服务名（如 /driver/command）
         */
        void RegisterModule(Module module, const std::string& service_name);

        /**
         * @brief 异步路由一条指令
         *
         * @param cmd 下游指令
         * @return future 结果（服务响应后兑现）
         */
        std::future<ModuleResult> RouteAsync(const DownstreamCmd& cmd) const;

        /**
         * @brief 批量路由并等待全部完成
         *
         * @param cmds 下游指令列表
         * @return 各指令结果（与入参顺序一致）
         */
        std::vector<ModuleResult> RouteAll(const std::vector<DownstreamCmd>& cmds) const;

    private:
        // 节点引用（创建服务客户端）
        rclcpp::Node& node_;

        // 模块 → 服务名
        std::unordered_map<Module, std::string> service_names_;

        // 模块 → 服务客户端（惰性创建，线程安全）
        mutable std::mutex clients_mutex_;
        mutable std::unordered_map<
            Module, rclcpp::Client<rus_sim_interfaces::srv::CommandService>::SharedPtr> clients_;
    };

}  // namespace RusSimApp
