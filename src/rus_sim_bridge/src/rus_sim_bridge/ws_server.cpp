#include "rus_sim_bridge/ws_server.hpp"

#include <chrono>
#include <cstring>
#include <iostream>

namespace rus_sim_bridge {

    using Channel = RusUtils::Channel;
    using RusUtils::channel_path;

    // ────────────────────────────────────────────────────────────────
    //  每会话用户数据
    // ────────────────────────────────────────────────────────────────
    namespace {
        struct PerSessionData {
            std::string command_buf;  // 收包缓冲（按 '}' 判定一条完整 JSON）
        };
    }

    // ────────────────────────────────────────────────────────────────
    //  libwebsockets 协议数组（需静态存储）
    //  同一回调；客户端以路径 /control /state /sensor 连接，
    //  回调里再按请求路径判定通道（见 LWS_CALLBACK_ESTABLISHED）。
    // ────────────────────────────────────────────────────────────────
    static const struct lws_protocols ws_protocols[] = {
        { .name = "control", .callback = WsServer::ws_callback,
          .per_session_data_size = sizeof(PerSessionData), .rx_buffer_size = 4096 },
        { .name = "state", .callback = WsServer::ws_callback,
          .per_session_data_size = sizeof(PerSessionData), .rx_buffer_size = 4096 },
        { .name = "sensor", .callback = WsServer::ws_callback,
          .per_session_data_size = sizeof(PerSessionData), .rx_buffer_size = 4096 },
        { nullptr, nullptr, 0, 0 }  // 终止
    };

    // ================================================================
    //  公开接口
    // ================================================================

    WsServer::~WsServer() { Stop(); }

    void WsServer::Start(int port, CommandHandler handler, LogFn log) {
        if (running_.load()) return;
        handler_ = std::move(handler);
        log_ = std::move(log);
        running_.store(true);

        auto log_msg = [this](int level, const std::string& msg) {
            if (log_) log_(level, msg);
            else {
                if (level == 2) std::cerr << "[WsServer] " << msg << std::endl;
                else std::cout << "[WsServer] " << msg << std::endl;
            }
        };

        int try_port = port;
        thread_ = std::thread([this, try_port, log_msg]() {
            for (int attempt = 0; attempt < 3; ++attempt) {
                int p = try_port + attempt;
                log_msg(0, "正在启动 ws://0.0.0.0:" + std::to_string(p) + " ...");

                lws_context_creation_info info{};
                info.port = p;
                info.protocols = ws_protocols;
                info.user = this;
                info.gid = -1;
                info.uid = -1;

                context_.reset(lws_create_context(&info));
                if (context_) {
                    port_ = p;
                    log_msg(0, "已启动 ws://0.0.0.0:" + std::to_string(p) +
                               "  (路径 /control /state /sensor)");
                    break;
                }
                log_msg(1, "端口 " + std::to_string(p) + " 不可用，尝试下一个...");
            }

            if (!context_) {
                log_msg(2, "所有端口均不可用，WebSocket 服务器启动失败");
                running_.store(false);
                return;
            }

            while (running_.load() && context_) {
                lws_service(context_.get(), 50);
                flush_scheduled();
            }

            context_.reset();
            running_.store(false);
        });

        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        if (!running_.load()) {
            log_msg(2, "启动失败，端口 " + std::to_string(port) + " 系列均被占用");
        }
    }

    void WsServer::Stop() {
        running_.store(false);
        if (thread_.joinable()) thread_.join();
        context_.reset();
    }

    void WsServer::BroadcastState(const std::string& json) {
        if (!running_.load()) return;
        {
            std::lock_guard lock(state_mutex_);
            last_state_json_ = json;  // 覆盖式，只保留最新
            ++state_gen_;
        }
        if (context_) lws_cancel_service(context_.get());  // 唤醒事件循环
    }

    void WsServer::BroadcastEvent(const std::string& json) {
        if (!running_.load()) return;
        {
            std::lock_guard lock(registry_mutex_);
            for (auto& [wsi, info] : sessions_) {
                if (info.channel == Channel::Control)
                    pending_replies_[info.id].push_back(json);
            }
        }
        if (context_) lws_cancel_service(context_.get());
    }

    void WsServer::BroadcastSensor(std::vector<uint8_t> frame) {
        if (!running_.load() || frame.empty()) return;
        {
            std::lock_guard lock(sensor_mutex_);
            // 覆盖式单槽：换出旧帧（读侧只做引用计数，不拷贝兆级字节）
            last_sensor_frame_ = std::make_shared<const std::vector<uint8_t>>(std::move(frame));
            ++sensor_gen_;
        }
        if (context_) lws_cancel_service(context_.get());  // 唤醒事件循环
    }

    // ================================================================
    //  内部
    // ================================================================

    void WsServer::enqueue_session(uint64_t session_id, const std::string& json) {
        std::lock_guard lock(registry_mutex_);
        auto it = sessions_by_id_.find(session_id);
        if (it == sessions_by_id_.end()) return;  // 会话已关闭，丢弃
        pending_replies_[session_id].push_back(json);
        if (context_) lws_cancel_service(context_.get());
    }

    void WsServer::flush_scheduled() {
        // 状态是「覆盖式」通道：每次唤醒都必须带上代次，只推没有推过的版本
        std::string state_json;
        uint64_t state_gen = 0;
        {
            std::lock_guard lock(state_mutex_);
            state_json = last_state_json_;
            state_gen = state_gen_;
        }

        uint64_t sensor_gen = 0;
        {
            std::lock_guard lock(sensor_mutex_);
            sensor_gen = sensor_gen_;
        }

        std::vector<lws*> to_wake;
        {
            std::lock_guard lock(registry_mutex_);
            for (auto& [wsi, info] : sessions_) {
                if (info.channel == Channel::State) {
                    if (!state_json.empty() && info.state_sent_gen != state_gen)
                        to_wake.push_back(wsi);
                } else if (info.channel == Channel::Sensor) {
                    if (sensor_gen != 0 && info.sensor_sent_gen != sensor_gen)
                        to_wake.push_back(wsi);
                } else {
                    auto it = pending_replies_.find(info.id);
                    if (it != pending_replies_.end() && !it->second.empty())
                        to_wake.push_back(wsi);
                }
            }
        }
        for (auto* w : to_wake) lws_callback_on_writable(w);
    }

    // ================================================================
    //  libwebsockets 协议回调（静态）
    // ================================================================

    int WsServer::ws_callback(lws* wsi, lws_callback_reasons reason,
                              void* user, void* in, size_t len) {
        lws_context* ctx = lws_get_context(wsi);
        if (!ctx) return -1;
        auto* server = static_cast<WsServer*>(lws_context_user(ctx));
        if (!server) return -1;
        auto* session = static_cast<PerSessionData*>(user);

        switch (reason) {
        case LWS_CALLBACK_ESTABLISHED: {
            new (session) PerSessionData();

            // 由请求路径判定通道（兼容子协议名，兜底 /control）
            Channel ch = Channel::Control;
            const char* proto_name = lws_get_protocol(wsi)->name;
            char uri[128] = {0};
            if (lws_hdr_copy(wsi, uri, sizeof(uri), WSI_TOKEN_GET_URI) > 0) {
                if (strstr(uri, "/state"))      ch = Channel::State;
                else if (strstr(uri, "/sensor")) ch = Channel::Sensor;
            } else if (proto_name) {
                if (!strcmp(proto_name, "state"))  ch = Channel::State;
                else if (!strcmp(proto_name, "sensor")) ch = Channel::Sensor;
            }

            {
                std::lock_guard lock(server->registry_mutex_);
                server->sessions_[wsi] = { server->next_session_id_++, ch };
                server->sessions_by_id_[server->sessions_[wsi].id] = wsi;
            }
            if (server->log_)
                server->log_(0, "客户端已连接 (channel=" + std::string(channel_path(ch)) + ")");
            break;
        }

        case LWS_CALLBACK_CLOSED: {
            {
                std::lock_guard lock(server->registry_mutex_);
                auto it = server->sessions_.find(wsi);
                if (it != server->sessions_.end()) {
                    server->sessions_by_id_.erase(it->second.id);
                    server->pending_replies_.erase(it->second.id);
                    server->sessions_.erase(it);
                }
            }
            session->~PerSessionData();
            if (server->log_) server->log_(0, "客户端已断开");
            break;
        }

        case LWS_CALLBACK_RECEIVE: {
            session->command_buf.append(static_cast<const char*>(in), len);
            // 一条完整 JSON 以 '}' 收尾
            if (!session->command_buf.empty() && session->command_buf.back() == '}') {
                RusUtils::CommandMessage cmd;
                std::string json = session->command_buf;
                session->command_buf.clear();

                uint64_t session_id = 0;
                {
                    std::lock_guard lock(server->registry_mutex_);
                    auto it = server->sessions_.find(wsi);
                    if (it != server->sessions_.end()) session_id = it->second.id;
                }

                if (RusUtils::ParseCommandMessage(json, cmd) && server->handler_) {
                    // reply 关联到本会话
                    auto reply = [server, session_id](const std::string& reply_json) {
                        server->enqueue_session(session_id, reply_json);
                    };
                    server->handler_(cmd, std::move(reply));
                } else if (server->handler_) {
                    server->enqueue_session(session_id,
                        RusUtils::SerializeResult(
                            RusUtils::ResultMessage::MakeReply(0, false, "malformed command")));
                }
            }
            break;
        }

        case LWS_CALLBACK_SERVER_WRITEABLE: {
            uint64_t session_id = 0;
            uint64_t sent_state_gen = 0;
            uint64_t sent_sensor_gen = 0;
            Channel ch = Channel::Control;
            {
                std::lock_guard lock(server->registry_mutex_);
                auto it = server->sessions_.find(wsi);
                if (it != server->sessions_.end()) {
                    session_id = it->second.id;
                    ch = it->second.channel;
                    sent_state_gen = it->second.state_sent_gen;
                    sent_sensor_gen = it->second.sensor_sent_gen;
                }
            }

            // ⚠️ lws 对同一次 lws_callback_on_writable() 请求会**重复回调**本回调
            // （实测一个 service 周期内可回调数百次），因此下面每个通道都必须按
            // 「代次」做幂等：本会话已推过的版本立刻 return，绝不重发。
            if (ch == Channel::State) {
                std::string json;
                uint64_t gen = 0;
                {
                    std::lock_guard lock(server->state_mutex_);
                    json = server->last_state_json_;
                    gen = server->state_gen_;
                }
                if (json.empty() || gen == 0 || sent_state_gen == gen) return 0;

                // 对端/内核发送缓冲已满：丢本帧（可丢帧通道），下一轮 flush 再推最新值
                if (lws_send_pipe_choked(wsi)) return 0;

                std::vector<unsigned char> buf(LWS_PRE + json.size());
                std::memcpy(buf.data() + LWS_PRE, json.data(), json.size());
                if (lws_write(wsi, buf.data() + LWS_PRE, json.size(), LWS_WRITE_TEXT) < 0)
                    return -1;

                std::lock_guard lock(server->registry_mutex_);
                auto it = server->sessions_.find(wsi);
                if (it != server->sessions_.end()) it->second.state_sent_gen = gen;
                return 0;
            }

            if (ch == Channel::Sensor) {
                // 一帧 = 一条 WS 二进制消息（覆盖式：只发最新一帧）
                std::shared_ptr<const std::vector<uint8_t>> frame;
                uint64_t gen = 0;
                {
                    std::lock_guard lock(server->sensor_mutex_);
                    frame = server->last_sensor_frame_;
                    gen = server->sensor_gen_;
                }
                if (!frame || frame->empty() || gen == 0 || sent_sensor_gen == gen) return 0;

                // 对端/内核发送缓冲已满：本帧直接丢（可丢帧通道），管道空了下一轮 flush 会重发最新帧
                if (lws_send_pipe_choked(wsi)) return 0;

                // 帧可到兆级：必须动态分配；整帧一次 lws_write，OS 未接受的部分
                // lws 会自动缓冲续发（见 libwebsockets/lws-write.h「Truncated Writes」），
                // 因此「一次 lws_write = 一条 WS 消息」，前端可据此做完整性自检。
                std::vector<unsigned char> buf(LWS_PRE + frame->size());
                std::memcpy(buf.data() + LWS_PRE, frame->data(), frame->size());
                int n = lws_write(wsi, buf.data() + LWS_PRE,
                                  frame->size(), LWS_WRITE_BINARY);
                if (n < 0) return -1;

                // 记录已发送的代次（丢帧由代次跳跃体现，不重发旧帧）
                std::lock_guard lock(server->registry_mutex_);
                auto it = server->sessions_.find(wsi);
                if (it != server->sessions_.end()) it->second.sensor_sent_gen = gen;
                return 0;
            }

            // control：取本会话待推送队列
            std::vector<std::string> queue;
            {
                std::lock_guard lock(server->registry_mutex_);
                auto it = server->pending_replies_.find(session_id);
                if (it != server->pending_replies_.end()) {
                    queue.swap(it->second);
                }
            }
            for (auto& json : queue) {
                if (json.size() > 16000) json.resize(16000);
                unsigned char buf[LWS_PRE + 16384];
                int n = json.copy(reinterpret_cast<char*>(buf) + LWS_PRE, 16384);
                if (n > 0) lws_write(wsi, buf + LWS_PRE, static_cast<size_t>(n), LWS_WRITE_TEXT);
            }
            break;
        }

        default:
            break;
        }
        return 0;
    }

}  // namespace rus_sim_bridge
