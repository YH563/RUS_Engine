#pragma once

#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <rus_sim_interfaces/msg/module_event.hpp>
#include <rus_sim_interfaces/msg/robot_state.hpp>
#include <rus_sim_interfaces/msg/sensor_frame.hpp>

#include <rus_sim_utils/protocol.hpp>

#include "rus_sim_bridge/command_dispatcher.hpp"
#include "rus_sim_bridge/ws_server.hpp"

namespace rus_sim_bridge {

    /**
     * @brief 前后端桥接节点（纯网关）
     *
     * 职责：
     * - 启动 WebSocket 多通道服务（/control /state /sensor）
     * - 指令接入：WS 事件循环线程收到 command → 入队
     * - 指令分发委托给 CommandDispatcher（解析 / 路由 / 扇出 / 回执）
     * - 子模块事件经 /module_events topic 上报，转为前端统一 event
     * - 状态流转 StateMessage 广播到 /state 通道
     * - 感知帧（SensorFrame）原样转成线格式广播到 /sensor 二进制通道
     *
     * 线程模型：
     *   - WS 事件循环线程：收到 command → 入队（on_frontend_command）
     *   - 执行器线程：定时器出队 → dispatcher_.Dispatch（异步路由，回执经 ReplyFn 返回）
     *   - 执行器线程：订阅回调 on_state / on_module_event / on_sensor → 推给 WsServer
     *     （WsServer 内部各自加锁并用 lws_cancel_service 唤醒事件循环，可跨线程调用）
     *
     * 生命周期：必须经 Create() 工厂创建（返回 shared_ptr）。
     *   dispatcher 需要节点的 shared_ptr，而 shared_ptr 只能在工厂 Create 中取得
     *   （构造函数内无法获得自身的 shared_ptr），故拆成：
     *   构造（建节点 / 起服务 / 订阅）+ init(self)（建 dispatcher / 定时器）。
     */
    class BridgeNode : public rclcpp::Node {
    public:
        /**
         * @brief 工厂创建：make_shared 托管后调 init()，一步到位
         */
        static std::shared_ptr<BridgeNode> Create(
            const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    private:
        explicit BridgeNode(const rclcpp::NodeOptions& options);
        void init(const std::shared_ptr<BridgeNode>& self);  // 创建 dispatcher（需要 self）

        using ModuleEvent = rus_sim_interfaces::msg::ModuleEvent;
        using SensorFrame = rus_sim_interfaces::msg::SensorFrame;

        void on_frontend_command(const RusUtils::CommandMessage& cmd, WsServer::ReplyFn reply);
        void drain_queue();   // 定时器：处理积压 command
        void on_state(const rus_sim_interfaces::msg::RobotState::SharedPtr msg);
        void on_module_event(const ModuleEvent::SharedPtr msg);
        void on_sensor(const SensorFrame::SharedPtr msg);

        // 指令队列（WS 线程入队 / 执行器线程出队）
        std::mutex queue_mutex_;
        std::deque<std::pair<RusUtils::CommandMessage, WsServer::ReplyFn>> pending_;

        // 指令解析 / 注册表路由 / 服务下发（独立类，本节点只做转发）
        std::unique_ptr<CommandDispatcher> dispatcher_;

        rclcpp::TimerBase::SharedPtr drain_timer_;
        rclcpp::TimerBase::SharedPtr timeout_timer_;
        rclcpp::Subscription<rus_sim_interfaces::msg::RobotState>::SharedPtr state_sub_;
        rclcpp::Subscription<ModuleEvent>::SharedPtr event_sub_;
        rclcpp::Subscription<SensorFrame>::SharedPtr sensor_sub_;

        WsServer ws_;
        double last_state_ts_ = -1.0;
        double state_rate_ = 0.0;
        bool forward_sensor_ = true;   // false = 不订阅感知流（/sensor 通道无数据）
    };

}  // namespace rus_sim_bridge
