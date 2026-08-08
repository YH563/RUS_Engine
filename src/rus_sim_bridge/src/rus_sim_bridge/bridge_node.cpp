#include "rus_sim_bridge/bridge_node.hpp"

#include <algorithm>
#include <chrono>
#include <functional>
#include <string>

namespace rus_sim_bridge {

    namespace {
        const std::vector<std::string>& kDriverPassthrough() {
            static const std::vector<std::string> cmds = {
                "movej", "movel", "servoj", "servo_cart",
                "start_jog", "stop_jog_decel", "stop_jog_immediate",
                "servo_start", "servo_end", "disconnect",
                "is_connected", "is_in_drag_teach", "robot_enable",
                "get_state", "is_motion_done", "run_file", "switch_driver",
                "set_time_speed", "get_time_speed", "get_sim_time",
                "step_once", "get_frame_rate",
            };
            return cmds;
        }
    }  // namespace

    // ================================================================
    //  构造
    // ================================================================

    BridgeNode::BridgeNode(const rclcpp::NodeOptions& options)
        : Node("bridge_node", options)
    {
        int ws_port = declare_parameter<int>("ws_port", 8765);
        int timeout_ms = declare_parameter<int>("timeout_ms", 5000);
        int drain_ms = declare_parameter<int>("drain_ms", 10);
        std::string state_topic = declare_parameter<std::string>("state_topic", "/driver/state");

        init_routing();

        // ── WebSocket 多通道服务 ──
        ws_.Start(ws_port,
            [this](const CommandMessage& cmd, WsServer::ReplyFn reply) {
                on_frontend_command(cmd, std::move(reply));
            },
            [this](int level, const std::string& msg) {
                switch (level) {
                    case 0:  RCLCPP_INFO(get_logger(), "%s", msg.c_str()); break;
                    case 1:  RCLCPP_WARN(get_logger(), "%s", msg.c_str()); break;
                    case 2:  RCLCPP_ERROR(get_logger(), "%s", msg.c_str()); break;
                }
            });

        // ── 订阅状态流 → 广播到 /state 通道 ──
        state_sub_ = create_subscription<rus_sim_interfaces::msg::RobotState>(
            state_topic, 10,
            std::bind(&BridgeNode::on_state, this, std::placeholders::_1));

        // ── 指令队列定时器（执行器线程出队 → 异步路由） ──
        drain_timer_ = create_wall_timer(
            std::chrono::milliseconds(drain_ms),
            std::bind(&BridgeNode::drain_queue, this));

        timeout_timer_ = create_wall_timer(
            std::chrono::milliseconds(200),
            std::bind(&BridgeNode::check_timeouts, this));

        RCLCPP_INFO(get_logger(), "BridgeNode 已启动 ws://0.0.0.0:%d (control/state/sensor)", ws_port);
    }

    // ================================================================
    //  路由表
    // ================================================================

    void BridgeNode::init_routing() {
        const std::string kDriver = "/driver/command";
        const std::string kData   = "/data/command";
        const std::string kPlan   = "/planning/command";

        auto D = [](const std::string& s, const std::string& c, std::vector<double> args = {}) {
            return Downstream{ s, c, std::move(args) };
        };

        // ── 高层意图 → 路由（可扇出多条） ──
        high_level_["connect"]            = { D(kDriver, "connect") };
        high_level_["shutdown"]           = { D(kPlan, "shutdown") };
        high_level_["pre_scan_start"]     = { D(kPlan, "pre_scan_start") };
        high_level_["pre_scan_end"]       = { D(kPlan, "pre_scan_end") };
        high_level_["set_start_pose"]     = { D(kPlan, "set_start_pose") };
        high_level_["set_end_pose"]       = { D(kPlan, "set_end_pose") };
        high_level_["plan"]               = { D(kPlan, "plan") };
        high_level_["execute"]            = { D(kPlan, "execute") };
        high_level_["stop"]               = { D(kPlan, "stop"), D(kDriver, "stop") };
        high_level_["pause"]              = { D(kPlan, "pause"), D(kDriver, "pause") };
        high_level_["resume"]             = { D(kPlan, "resume"), D(kDriver, "resume") };
        high_level_["reset"]              = { D(kPlan, "reset") };
        high_level_["query_prescan_done"] = { D(kPlan, "query_prescan_done") };
        high_level_["query_motion_done"]  = { D(kDriver, "is_motion_done") };

        high_level_["record_start"]       = { D(kData, "record_start") };
        high_level_["record_stop"]        = { D(kData, "record_stop") };
        high_level_["playback_start"]     = { D(kData, "playback_start") };
        high_level_["playback_stop"]      = { D(kData, "playback_stop") };
        high_level_["playback_pause"]     = { D(kData, "playback_pause") };
        high_level_["playback_resume"]    = { D(kData, "playback_resume") };
        high_level_["playback_set_speed"] = { D(kData, "playback_set_speed") };
        high_level_["playback_seek"]      = { D(kData, "playback_seek") };
        high_level_["playback_step"]      = { D(kData, "playback_step") };
        high_level_["playback_set_loop"]  = { D(kData, "playback_set_loop") };
        high_level_["playback_get_info"]  = { D(kData, "playback_get_info") };

        // ── 底层驱动透传集 ──
        driver_passthrough_ = kDriverPassthrough();
    }

    // ================================================================
    //  command 处理（WS 线程入队 / 执行器线程异步路由）
    // ================================================================

    void BridgeNode::on_frontend_command(const CommandMessage& cmd, WsServer::ReplyFn reply) {
        std::lock_guard lock(queue_mutex_);
        pending_.emplace_back(cmd, std::move(reply));
    }

    void BridgeNode::drain_queue() {
        std::pair<CommandMessage, WsServer::ReplyFn> item;
        bool has = false;
        {
            std::lock_guard lock(queue_mutex_);
            if (!pending_.empty()) {
                item = std::move(pending_.front());
                pending_.pop_front();
                has = true;
            }
        }
        if (has) route(item.first, item.second);
    }

    void BridgeNode::route(const CommandMessage& cmd, const WsServer::ReplyFn& reply) {
        const double timeout_s = static_cast<double>(get_parameter("timeout_ms").as_int()) / 1000.0;

        auto make_ctx = [&](size_t pending_count) {
            auto ctx = std::make_shared<FanOutContext>();
            ctx->reply = reply;
            ctx->request_id = cmd.id;
            ctx->pending = pending_count;
            ctx->deadline = now().seconds() + timeout_s;
            return ctx;
        };

        // 1) 高层路由（可扇出多条）
        auto it = high_level_.find(cmd.cmd);
        if (it != high_level_.end()) {
            auto ctx = make_ctx(it->second.size());
            active_.push_back(ctx);
            for (const auto& d : it->second) {
                // 高层指令 args 为空 → 透传前端 args；路由表指定固定 args 则用之
                std::vector<double> args = d.args.empty() ? cmd.args : d.args;
                call_downstream(d, args, ctx);
            }
            return;
        }

        // 2) 驱动透传
        if (std::find(driver_passthrough_.begin(), driver_passthrough_.end(), cmd.cmd)
            != driver_passthrough_.end()) {
            auto ctx = make_ctx(1);
            active_.push_back(ctx);
            call_downstream(Downstream{ "/driver/command", cmd.cmd, {} }, cmd.args, ctx);
            return;
        }

        // 3) 未知指令
        ReplyMessage fail;
        fail.id = cmd.id;
        fail.success = false;
        fail.message = "unknown command: " + cmd.cmd;
        reply(serialize_reply(fail));
    }

    void BridgeNode::call_downstream(const Downstream& d, const std::vector<double>& args,
                                     const std::shared_ptr<FanOutContext>& ctx) {
        auto it = clients_.find(d.service);
        if (it == clients_.end()) {
            it = clients_.emplace(d.service, create_client<CommandService>(d.service)).first;
        }
        auto& client = it->second;

        if (!client->service_is_ready()) {
            // 模块未启动 → 立即失败，避免悬挂
            RCLCPP_WARN(get_logger(), "服务 %s 不可用，指令 %s 失败", d.service.c_str(), d.command.c_str());
            finish_fanout(ctx, false, "service unavailable: " + d.service, {});
            return;
        }

        auto req = std::make_shared<CommandService::Request>();
        req->command = d.command;
        req->args = args;

        client->async_send_request(req,
            [this, ctx](rclcpp::Client<CommandService>::SharedFuture future) {
                bool success = false;
                std::string message;
                std::vector<double> result;
                try {
                    auto res = future.get();
                    success = res->success;
                    message = res->message;
                    result = res->result;
                } catch (const std::exception& e) {
                    message = std::string("service exception: ") + e.what();
                }
                finish_fanout(ctx, success, message, std::move(result));
            });
    }

    void BridgeNode::finish_fanout(const std::shared_ptr<FanOutContext>& ctx, bool success,
                                   const std::string& message, std::vector<double> result) {
        if (ctx->done) return;
        if (!success) {
            ctx->all_success = false;
            if (ctx->message.empty()) ctx->message = message;
        }
        // 成功路径：各模块返回结果依次拼接（v0.1 简化）
        for (double v : result) ctx->result.push_back(v);

        --ctx->pending;
        if (ctx->pending == 0) {
            ctx->done = true;
            ReplyMessage reply;
            reply.id = ctx->request_id;
            reply.success = ctx->all_success;
            reply.message = ctx->message.empty() ? "ok" : ctx->message;
            reply.result = ctx->result;
            ctx->reply(serialize_reply(reply));
        }
    }

    void BridgeNode::check_timeouts() {
        const double now_s = now().seconds();
        for (auto& w : active_) {
            auto ctx = w.lock();
            if (ctx && !ctx->done && now_s > ctx->deadline) {
                finish_fanout(ctx, false, "timeout", {});
            }
        }
        active_.erase(std::remove_if(active_.begin(), active_.end(),
            [](const std::weak_ptr<FanOutContext>& w) { return w.expired(); }),
            active_.end());
    }

    // ================================================================
    //  状态流
    // ================================================================

    void BridgeNode::on_state(const rus_sim_interfaces::msg::RobotState::SharedPtr msg) {
        StateMessage s;
        s.timestamp = msg->timestamp;
        s.joint_pos = msg->joint_pos;
        s.joint_vel = msg->joint_vel;
        s.joint_acc = msg->joint_acc;
        s.effort = msg->effort;
        s.flange_pos = msg->flange_pos;

        if (last_state_ts_ > 0.0 && msg->timestamp > last_state_ts_) {
            double dt = msg->timestamp - last_state_ts_;
            if (dt > 0.0 && dt < 1.0) state_rate_ = 1.0 / dt;
        }
        last_state_ts_ = msg->timestamp;
        s.frame_rate = state_rate_;

        ws_.BroadcastState(serialize_state(s));
    }

}  // namespace rus_sim_bridge
