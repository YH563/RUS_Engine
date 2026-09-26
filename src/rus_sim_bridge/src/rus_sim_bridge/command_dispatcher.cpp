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
        registry_.Register(CmdName::kSetStartPose,     {Module::PLANNING});
        registry_.Register(CmdName::kSetEndPose,       {Module::PLANNING});
        registry_.Register(CmdName::kPlan,             {Module::PLANNING});
        registry_.Register(CmdName::kExecute,          {Module::PLANNING});

        // ── perception（点云预扫查 / 预处理）──
        // pre_scan_* 由 perception 全权处理（采集/处理/发布 /preprocessed_cloud），
        // planning 通过订阅 pre_scan_done 事件获取完成标记。
        registry_.Register(CmdName::kPreScanStart,     {Module::PERCEPTION});
        registry_.Register(CmdName::kPreScanEnd,       {Module::PERCEPTION});
        registry_.Register(CmdName::kQueryPreScanDone, {Module::PERCEPTION});

        // ── driver（驱动） ──
        registry_.Register(CmdName::kConnect,          {Module::DRIVER});
        registry_.Register(CmdName::kDisconnect,       {Module::DRIVER});
        registry_.Register(CmdName::kIsConnected,      {Module::DRIVER});
        registry_.Register(CmdName::kIsInDragTeach,    {Module::DRIVER});
        registry_.Register(CmdName::kRobotEnable,      {Module::DRIVER});
        registry_.Register(CmdName::kGetState,         {Module::DRIVER});
        registry_.Register(CmdName::kIsMotionDone,     {Module::DRIVER});
        registry_.Register(CmdName::kSwitchDriver,     {Module::DRIVER});
        registry_.Register(CmdName::kGetDriverType,    {Module::DRIVER});
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

        // ── 工具坐标系 / 标定（标定计算在驱动内部完成） ──
        registry_.Register(CmdName::kSetToolCalibPoint, {Module::DRIVER});
        registry_.Register(CmdName::kComputeToolCalib,  {Module::DRIVER});
        registry_.Register(CmdName::kSetToolCoord,      {Module::DRIVER});
        registry_.Register(CmdName::kSetToolIndex,      {Module::DRIVER});
        registry_.Register(CmdName::kGetToolCoords,     {Module::DRIVER});

        // ── 业务 / 状态指令（默认自动模式：planning 持有动作状态；set_mode 可切手动直控 driver）──
        // stop = 急停：安全关键，必须直接到达 driver；自动模式同时扇出 planning 停伺服循环。
        registry_.Register(CmdName::kStop,            {Module::PLANNING, Module::DRIVER});
        registry_.Register(CmdName::kPause,           {Module::PLANNING});
        registry_.Register(CmdName::kResume,          {Module::PLANNING});
        registry_.Register(CmdName::kReset,           {Module::PLANNING});
        registry_.Register(CmdName::kQueryMotionDone, {Module::PLANNING});

        // ── 回放（rus_sim_recorder_replay：离线复盘，与在线链路无耦合）──
        // 全部直达 replayer；不参与模式切换（手动/自动与回放无关）。
        registry_.Register(CmdName::kReplayLoad,     {Module::REPLAYER});
        registry_.Register(CmdName::kReplayList,     {Module::REPLAYER});
        registry_.Register(CmdName::kReplayStart,    {Module::REPLAYER});
        registry_.Register(CmdName::kReplayPause,    {Module::REPLAYER});
        registry_.Register(CmdName::kReplayResume,   {Module::REPLAYER});
        registry_.Register(CmdName::kReplayStop,     {Module::REPLAYER});
        registry_.Register(CmdName::kReplaySeek,     {Module::REPLAYER});
        registry_.Register(CmdName::kReplaySetSpeed, {Module::REPLAYER});
        registry_.Register(CmdName::kReplayStep,     {Module::REPLAYER});
        registry_.Register(CmdName::kReplayStatus,   {Module::REPLAYER});

        // ── 录制控制（recorder_node：运行期开 / 关落盘）──
        // 同样直达、不参与模式切换（录制与手动/自动无关）。
        registry_.Register(CmdName::kRecorderStart,  {Module::RECORDER});
        registry_.Register(CmdName::kRecorderStop,   {Module::RECORDER});
        registry_.Register(CmdName::kRecorderStatus, {Module::RECORDER});

        // ── 本地处理（不转发下游） ──
        registry_.Register(CmdName::kShutdown, {});
        registry_.Register(CmdName::kSetMode, {});   // 模式切换：0=手动, 1=自动（修改上述扇出目标）
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

        if (cmd.cmd == CmdName::kSetMode) {
            // 0=手动（直控 driver），1=自动（planning 协调，默认）
            bool auto_mode = cmd.args.empty() ? true : (static_cast<int>(cmd.args[0]) != 0);
            apply_mode(auto_mode);
            // 切到手动时通知 planning 停止残留扫描（避免自动任务悬挂）
            if (!auto_mode) {
                send_raw_to_module(RusUtils::Module::PLANNING, std::string(CmdName::kStop), {});
            }
            reply(SerializeResult(ResultMessage::MakeReply(cmd.id, true,
                auto_mode ? "mode: auto" : "mode: manual")));
            return;
        }

        reply(SerializeResult(ResultMessage::MakeReply(cmd.id, true, "ok")));
    }

    // ================================================================
    //  模式切换：修改模式相关指令的扇出目标
    // ================================================================

    void CommandDispatcher::apply_mode(bool auto_mode) {
        using namespace RusUtils;

        if (auto_mode) {
            // 自动：planning 持有动作状态，暂停/恢复/复位/查询走 planning；急停同时直达 driver
            registry_.SetTargets(CmdName::kStop,            {Module::PLANNING, Module::DRIVER});
            registry_.SetTargets(CmdName::kPause,           {Module::PLANNING});
            registry_.SetTargets(CmdName::kResume,          {Module::PLANNING});
            registry_.SetTargets(CmdName::kReset,           {Module::PLANNING});
            registry_.SetTargets(CmdName::kQueryMotionDone, {Module::PLANNING});
        } else {
            // 手动：直控 driver，不经 planning（避免重复转发 / 状态误判）
            registry_.SetTargets(CmdName::kStop,            {Module::DRIVER});
            registry_.SetTargets(CmdName::kPause,           {Module::DRIVER});
            registry_.SetTargets(CmdName::kResume,          {Module::DRIVER});
            registry_.SetTargets(CmdName::kReset,           {Module::DRIVER});
            registry_.SetTargets(CmdName::kQueryMotionDone, {Module::DRIVER});
        }
        auto_mode_ = auto_mode;
        RCLCPP_INFO(node_->get_logger(), "路由模式切换为 %s", auto_mode ? "自动" : "手动");
    }

    // ================================================================
    //  单向下发（fire-and-forget，不参与扇出回执）
    // ================================================================

    void CommandDispatcher::send_raw_to_module(RusUtils::Module module,
                                               const std::string& cmd,
                                               const std::vector<double>& args) {
        std::string service(RusUtils::module_service_name(module));
        auto it = clients_.find(service);
        if (it == clients_.end()) {
            it = clients_.emplace(service,
                node_->create_client<CommandService>(service)).first;
        }
        auto& client = it->second;
        if (!client->service_is_ready()) {
            RCLCPP_WARN(node_->get_logger(), "服务 %s 不可用，无法下发 %s", service.c_str(), cmd.c_str());
            return;
        }
        auto req = std::make_shared<CommandService::Request>();
        req->client_id = 0;
        req->command = cmd;
        req->args = args;
        client->async_send_request(req);
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
                std::vector<std::string> strings;
                try {
                    auto res = future.get();
                    success = res->success;
                    message = res->message;
                    result = res->result;
                    strings = res->strings;
                } catch (const std::exception& e) {
                    message = std::string("service exception: ") + e.what();
                }
                finish_fanout(ctx, success, message, std::move(result), std::move(strings));
            });
    }

    void CommandDispatcher::finish_fanout(const std::shared_ptr<FanOutContext>& ctx,
                                          bool success, const std::string& message,
                                          std::vector<double> result,
                                          std::vector<std::string> strings) {
        if (ctx->done) return;
        if (!success) {
            ctx->all_success = false;
            if (ctx->message.empty()) ctx->message = message;
        }
        // 成功路径：各模块返回结果依次拼接（数值 + 文本）
        for (double v : result) ctx->result.push_back(v);
        for (auto& s : strings) ctx->strings.push_back(std::move(s));

        --ctx->pending;
        if (ctx->pending == 0) {
            ctx->done = true;
            auto reply = RusUtils::ResultMessage::MakeReply(ctx->request_id, ctx->all_success,
                ctx->message.empty() ? "ok" : ctx->message, ctx->result, ctx->strings);
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
