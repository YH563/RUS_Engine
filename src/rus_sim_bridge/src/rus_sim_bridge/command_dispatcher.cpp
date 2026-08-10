#include "rus_sim_bridge/command_dispatcher.hpp"

#include <algorithm>
#include <functional>
#include <string>
#include <utility>

namespace rus_sim_bridge {

    // ================================================================
    //  构造：读取参数 + 初始化路由注册表
    // ================================================================

    CommandDispatcher::CommandDispatcher(std::shared_ptr<rclcpp::Node> node)
        : node_(std::move(node))
    {
        timeout_ms_ = node_->declare_parameter<int>("timeout_ms", 5000);
        init_routing();
    }

    // ================================================================
    //  路由注册表
    // ================================================================

    void CommandDispatcher::init_routing() {
        using namespace RusUtils;

        // ── planning（规划 / 执行） ──
        registry_.Register(CmdName::kPreScanStart,     {Module::PLANNING});
        registry_.Register(CmdName::kPreScanEnd,       {Module::PLANNING});
        registry_.Register(CmdName::kSetStartPose,     {Module::PLANNING});
        registry_.Register(CmdName::kSetEndPose,       {Module::PLANNING});
        registry_.Register(CmdName::kPlan,             {Module::PLANNING});
        registry_.Register(CmdName::kExecute,          {Module::PLANNING});
        registry_.Register(CmdName::kQueryPreScanDone, {Module::PLANNING});

        // ── driver（驱动） ──
        registry_.Register(CmdName::kConnect,          {Module::DRIVER});
        registry_.Register(CmdName::kDisconnect,       {Module::DRIVER});
        registry_.Register(CmdName::kIsConnected,      {Module::DRIVER});
        registry_.Register(CmdName::kIsInDragTeach,    {Module::DRIVER});
        registry_.Register(CmdName::kRobotEnable,      {Module::DRIVER});
        registry_.Register(CmdName::kGetState,         {Module::DRIVER});
        registry_.Register(CmdName::kIsMotionDone,     {Module::DRIVER});
        registry_.Register(CmdName::kSwitchDriver,     {Module::DRIVER});
        registry_.Register(CmdName::kMoveJ,            {Module::DRIVER});
        registry_.Register(CmdName::kMoveL,            {Module::DRIVER});
        registry_.Register(CmdName::kServoJ,           {Module::DRIVER});
        registry_.Register(CmdName::kServoCart,        {Module::DRIVER});
        registry_.Register(CmdName::kStartJog,         {Module::DRIVER});
        registry_.Register(CmdName::kStopJogDecel,     {Module::DRIVER});
        registry_.Register(CmdName::kStopJogImmediate, {Module::DRIVER});
        registry_.Register(CmdName::kServoStart,       {Module::DRIVER});
        registry_.Register(CmdName::kServoEnd,         {Module::DRIVER});
        registry_.Register(CmdName::kRunFile,          {Module::DRIVER});
        registry_.Register(CmdName::kSetTimeSpeed,     {Module::DRIVER});
        registry_.Register(CmdName::kGetTimeSpeed,     {Module::DRIVER});
        registry_.Register(CmdName::kGetSimTime,       {Module::DRIVER});
        registry_.Register(CmdName::kStepOnce,         {Module::DRIVER});
        registry_.Register(CmdName::kGetFrameRate,     {Module::DRIVER});

        // ── 扇出：planning + driver 都要收到 ──
        registry_.Register(CmdName::kStop,            {Module::PLANNING, Module::DRIVER});
        registry_.Register(CmdName::kPause,           {Module::PLANNING, Module::DRIVER});
        registry_.Register(CmdName::kResume,          {Module::PLANNING, Module::DRIVER});
        registry_.Register(CmdName::kReset,           {Module::PLANNING, Module::DRIVER});
        registry_.Register(CmdName::kQueryMotionDone, {Module::PLANNING, Module::DRIVER});

        // ── 本地处理（不转发下游） ──
        registry_.Register(CmdName::kShutdown, {});
    }

    // ================================================================
    //  分发入口
    // ================================================================

    void CommandDispatcher::Dispatch(const RusUtils::CommandMessage& cmd,
                                     WsServer::ReplyFn reply) {
        using namespace RusUtils;

        // 1) 解析为类型化指令（校验合法性）
        std::string err;
        auto parsed = Cmd::ParseCommand(cmd.cmd, cmd.args, err);
        if (!parsed) {
            reply(SerializeResult(ResultMessage::MakeReply(cmd.id, false, err)));
            return;
        }

        // 2) 按注册表分发
        if (!registry_.IsRegistered(cmd.cmd)) {
            reply(SerializeResult(ResultMessage::MakeReply(cmd.id, false,
                "unknown command: " + cmd.cmd)));
            return;
        }

        if (registry_.IsLocal(cmd.cmd)) {
            handle_local(cmd, reply);
            return;
        }

        // 3) 扇出到目标模块
        auto targets = registry_.TargetsOf(cmd.cmd);
        auto ctx = std::make_shared<FanOutContext>();
        ctx->reply = std::move(reply);
        ctx->request_id = cmd.id;
        ctx->pending = targets.size();
        ctx->deadline = node_->now().seconds() + static_cast<double>(timeout_ms_) / 1000.0;
        active_.push_back(ctx);
        for (auto m : targets) call_downstream(m, cmd, ctx);
    }

    // ================================================================
    //  本地指令（bridge 自身处理，不转发下游）
    // ================================================================

    void CommandDispatcher::handle_local(const RusUtils::CommandMessage& cmd,
                                         const WsServer::ReplyFn& reply) {
        using namespace RusUtils;

        if (cmd.cmd == CmdName::kShutdown) {
            reply(SerializeResult(ResultMessage::MakeReply(cmd.id, true, "shutting down")));
            // 延迟触发关闭，确保 reply 先推送
            rclcpp::shutdown();
            return;
        }

        reply(SerializeResult(ResultMessage::MakeReply(cmd.id, true, "ok")));
    }

    // ================================================================
    //  下游调用
    // ================================================================

    void CommandDispatcher::call_downstream(RusUtils::Module module,
                                            const RusUtils::CommandMessage& cmd,
                                            const std::shared_ptr<FanOutContext>& ctx) {
        std::string service(RusUtils::module_service_name(module));
        auto it = clients_.find(service);
        if (it == clients_.end()) {
            it = clients_.emplace(service,
                node_->create_client<CommandService>(service)).first;
        }
        auto& client = it->second;

        if (!client->service_is_ready()) {
            // 模块未启动 → 立即失败，避免悬挂
            RCLCPP_WARN(node_->get_logger(), "服务 %s 不可用，指令 %s 失败",
                        service.c_str(), cmd.cmd.c_str());
            finish_fanout(ctx, false, "service unavailable: " + service, {});
            return;
        }

        auto req = std::make_shared<CommandService::Request>();
        req->client_id = cmd.id;
        req->command = cmd.cmd;
        req->args = cmd.args;

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

    void CommandDispatcher::finish_fanout(const std::shared_ptr<FanOutContext>& ctx,
                                          bool success, const std::string& message,
                                          std::vector<double> result) {
        if (ctx->done) return;
        if (!success) {
            ctx->all_success = false;
            if (ctx->message.empty()) ctx->message = message;
        }
        // 成功路径：各模块返回结果依次拼接
        for (double v : result) ctx->result.push_back(v);

        --ctx->pending;
        if (ctx->pending == 0) {
            ctx->done = true;
            auto reply = RusUtils::ResultMessage::MakeReply(ctx->request_id, ctx->all_success,
                ctx->message.empty() ? "ok" : ctx->message, ctx->result);
            ctx->reply(RusUtils::SerializeResult(reply));
        }
    }

    // ================================================================
    //  超时检测
    // ================================================================

    void CommandDispatcher::CheckTimeouts() {
        const double now_s = node_->now().seconds();
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

}  // namespace rus_sim_bridge
