#include "rus_sim_bridge/bridge_node.hpp"

#include <chrono>
#include <functional>
#include <string>

namespace rus_sim_bridge {

    // ================================================================
    //  工厂创建
    // ================================================================

    std::shared_ptr<BridgeNode> BridgeNode::Create(const rclcpp::NodeOptions& options) {
        // make_shared 托管后，把自身的 shared_ptr 传给 init 创建 dispatcher
        auto node = std::shared_ptr<BridgeNode>(new BridgeNode(options));
        node->init(node);
        return node;
    }

    // ================================================================
    //  构造：建节点、起 WS 服务、订阅子模块
    // ================================================================

    BridgeNode::BridgeNode(const rclcpp::NodeOptions& options)
        : Node("bridge_node", options)
    {
        int ws_port = declare_parameter<int>("ws_port", 8765);
        std::string state_topic = declare_parameter<std::string>("state_topic", "/driver/state");

        // ── WebSocket 多通道服务 ──
        ws_.Start(ws_port,
            [this](const RusUtils::CommandMessage& cmd, WsServer::ReplyFn reply) {
                on_frontend_command(cmd, std::move(reply));
            },
            [this](int level, const std::string& msg) {
                switch (level) {
                    case 0:  RCLCPP_INFO(get_logger(), "%s", msg.c_str()); break;
                    case 1:  RCLCPP_WARN(get_logger(), "%s", msg.c_str()); break;
                    case 2:  RCLCPP_ERROR(get_logger(), "%s", msg.c_str()); break;
                }
            });

        // ── 订阅子模块事件 → 广播为前端 event ──
        event_sub_ = create_subscription<ModuleEvent>(
            "/module_events", 10,
            std::bind(&BridgeNode::on_module_event, this, std::placeholders::_1));

        // ── 订阅状态流 → 广播到 /state 通道 ──
        state_sub_ = create_subscription<rus_sim_interfaces::msg::RobotState>(
            state_topic, 10,
            std::bind(&BridgeNode::on_state, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "BridgeNode 已启动 ws://0.0.0.0:%d (control/state/sensor)", ws_port);
    }

    // ================================================================
    //  init：创建 dispatcher 与定时器（需要 self 提供 shared_ptr）
    // ================================================================

    void BridgeNode::init(const std::shared_ptr<BridgeNode>& self) {
        int drain_ms = declare_parameter<int>("drain_ms", 10);

        // 指令下发器（解析 / 路由 / 扇出 / 超时检测）
        dispatcher_ = std::make_unique<CommandDispatcher>(self);

        // ── 指令队列定时器（执行器线程出队 → 分发） ──
        drain_timer_ = create_wall_timer(
            std::chrono::milliseconds(drain_ms),
            std::bind(&BridgeNode::drain_queue, this));

        // ── 下游服务超时检测（委托给 dispatcher） ──
        timeout_timer_ = create_wall_timer(
            std::chrono::milliseconds(200),
            [this]() { dispatcher_->CheckTimeouts(); });
    }

    // ================================================================
    //  command 接入（WS 线程入队 / 执行器线程分发）
    // ================================================================

    void BridgeNode::on_frontend_command(const RusUtils::CommandMessage& cmd, WsServer::ReplyFn reply) {
        std::lock_guard lock(queue_mutex_);
        pending_.emplace_back(cmd, std::move(reply));
    }

    void BridgeNode::drain_queue() {
        std::pair<RusUtils::CommandMessage, WsServer::ReplyFn> item;
        bool has = false;
        {
            std::lock_guard lock(queue_mutex_);
            if (!pending_.empty()) {
                item = std::move(pending_.front());
                pending_.pop_front();
                has = true;
            }
        }
        if (has) dispatcher_->Dispatch(item.first, item.second);
    }

    // ================================================================
    //  事件流（子模块 → bridge → 前端 event）
    // ================================================================

    void BridgeNode::on_module_event(const ModuleEvent::SharedPtr evt) {
        auto r = RusUtils::ResultMessage::MakeEvent(evt->event, evt->client_id,
                                                    evt->success, evt->message, evt->result);
        ws_.BroadcastEvent(RusUtils::SerializeResult(r));
    }

    // ================================================================
    //  状态流
    // ================================================================

    void BridgeNode::on_state(const rus_sim_interfaces::msg::RobotState::SharedPtr msg) {
        RusUtils::StateMessage s;
        s.timestamp = rclcpp::Time(msg->header.stamp).seconds();
        s.joint_pos = msg->joint_pos;
        s.joint_vel = msg->joint_vel;
        s.joint_acc = msg->joint_acc;
        s.effort = msg->effort;
        s.flange_pos = msg->flange_pos;

        const double ts = rclcpp::Time(msg->header.stamp).seconds();
        if (last_state_ts_ > 0.0 && ts > last_state_ts_) {
            double dt = ts - last_state_ts_;
            if (dt > 0.0 && dt < 1.0) state_rate_ = 1.0 / dt;
        }
        last_state_ts_ = ts;
        s.frame_rate = state_rate_;

        ws_.BroadcastState(RusUtils::SerializeState(s));
    }

}  // namespace rus_sim_bridge
