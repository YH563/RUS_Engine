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
        std::string state_json;
        {
            std::lock_guard lock(state_mutex_);
            state_json = last_state_json_;
        }

        std::vector<lws*> to_wake;
        {
            std::lock_guard lock(registry_mutex_);
            for (auto& [wsi, info] : sessions_) {
                if (info.channel == Channel::State) {
                    if (!state_json.empty()) to_wake.push_back(wsi);
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
            Channel ch = Channel::Control;
            {
                std::lock_guard lock(server->registry_mutex_);
                auto it = server->sessions_.find(wsi);
                if (it != server->sessions_.end()) {
                    session_id = it->second.id;
                    ch = it->second.channel;
                }
            }

            if (ch == Channel::State) {
                std::string json;
                {
                    std::lock_guard lock(server->state_mutex_);
                    json = server->last_state_json_;
                }
                if (!json.empty()) {
                    unsigned char buf[LWS_PRE + 16384];
                    int n = json.copy(reinterpret_cast<char*>(buf) + LWS_PRE, 16384);
                    if (n > 0) lws_write(wsi, buf + LWS_PRE, static_cast<size_t>(n), LWS_WRITE_TEXT);
                }
                return 0;
            }

            // control / sensor：取本会话待推送队列
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
