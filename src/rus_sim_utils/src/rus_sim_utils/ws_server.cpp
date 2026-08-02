#include "rus_sim_utils/ws_server.hpp"

#include <cstring>
#include <iostream>
#include <sstream>

#include <libwebsockets.h>

namespace RusUtils {

    // ── 每个 WS 连接持有的用户数据 ──
    struct PerSessionData {
        bool pending_command = false;
        std::string command_buf;
    };

    // ── libwebsockets 协议数组（需静态存储） ──
    static const struct lws_protocols ws_protocols[] = {
        {
            .name = "robot-monitor",
            .callback = WsServer::ws_callback,
            .per_session_data_size = sizeof(PerSessionData),
            .rx_buffer_size = 4096,
        },
        { nullptr, nullptr, 0, 0 }  // 终止
    };

    // ============================================================
    //  公开接口
    // ============================================================

    WsServer::~WsServer() { Stop(); }

    void WsServer::Start(int port, DispatchFn dispatch, LogFn log) {
        if (running_) return;
        dispatch_ = std::move(dispatch);
        log_ = std::move(log);
        running_ = true;

        auto log_msg = [this](int level, const std::string& msg) {
            if (log_) log_(level, msg);
            else {
                if (level == 2) std::cerr << "[WsServer] " << msg << std::endl;
                else std::cout << "[WsServer] " << msg << std::endl;
            }
        };

        // 端口自动回退：尝试 port, port+1, port+2
        int try_port = port;
        int max_attempts = 3;
        thread_ = std::thread([this, try_port, max_attempts, log_msg]() {
            for (int attempt = 0; attempt < max_attempts; ++attempt) {
                int p = try_port + attempt;
                log_msg(0, "正在启动 ws://0.0.0.0:" + std::to_string(p) + " ...");

                lws_context_creation_info info{};
                info.port = p;
                info.protocols = ws_protocols;
                info.user = this;
                info.gid = -1;
                info.uid = -1;

                context_ = lws_create_context(&info);
                if (context_) {
                    port_ = p;
                    log_msg(0, "已启动 ws://0.0.0.0:" + std::to_string(p) + "  (等待客户端...)");
                    break;
                }
                log_msg(1, "端口 " + std::to_string(p) + " 不可用，尝试下一个...");
            }

            if (!context_) {
                log_msg(2, "所有端口均不可用，WebSocket 服务器启动失败");
                running_ = false;
                return;
            }

            while (running_ && context_) {
                lws_service(context_, 50);
                // 每次唤醒后通知所有客户端去取最新状态
                lws_callback_on_writable_all_protocol(context_, &ws_protocols[0]);
            }

            lws_context_destroy(context_);
            context_ = nullptr;
        });

        // 等待线程启动
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        if (!running_) {
            log_msg(2, "启动失败，端口 " + std::to_string(port) + " 系列均被占用");
        }
    }

    void WsServer::Stop() {
        running_ = false;
        if (thread_.joinable()) thread_.join();
        if (context_) {
            lws_context_destroy(context_);
            context_ = nullptr;
        }
    }

    void WsServer::Broadcast(const std::string& json) {
        if (!running_) return;
        // 单状态覆盖：直接替换，新数据覆盖旧数据
        {
            std::lock_guard lock(state_mutex_);
            last_state_json_ = json;
        }
        if (context_) {
            lws_cancel_service(context_);  // 唤醒事件循环
        }
    }

    // ============================================================
    //  libwebsockets 事件循环
    // ============================================================

    // ============================================================
    //  libwebsockets 协议回调（静态）
    // ============================================================

    int WsServer::ws_callback(lws* wsi, lws_callback_reasons reason,
                            void* user, void* in, size_t len) {
        auto* ctx = lws_get_context(wsi);
        if (!ctx) return -1;
        auto* server = static_cast<WsServer*>(lws_context_user(ctx));
        if (!server) return -1;
        auto* session = static_cast<PerSessionData*>(user);

        switch (reason) {
        case LWS_CALLBACK_ESTABLISHED: {
            new (session) PerSessionData();
            if (server->log_) server->log_(0, "客户端已连接");
            break;
        }

        case LWS_CALLBACK_CLOSED: {
            session->~PerSessionData();
            if (server->log_) server->log_(0, "客户端已断开");
            break;
        }

        case LWS_CALLBACK_RECEIVE: {
            // 收到客户端消息 → 解析为指令
            std::string msg(static_cast<const char*>(in), len);
            session->command_buf += msg;

            if (!msg.empty() && msg.back() == '}') {
                std::string json = session->command_buf;
                session->command_buf.clear();

                auto find_str = [&](const std::string& key) -> std::string {
                    auto pos = json.find("\"" + key + "\"");
                    if (pos == std::string::npos) return "";
                    pos = json.find('"', pos + key.size() + 3);
                    if (pos == std::string::npos) return "";
                    auto end = json.find('"', pos + 1);
                    if (end == std::string::npos) return "";
                    return json.substr(pos + 1, end - pos - 1);
                };
                auto find_arr = [&](const std::string& key) -> std::vector<double> {
                    auto pos = json.find("\"" + key + "\"");
                    if (pos == std::string::npos) return {};
                    pos = json.find('[', pos);
                    if (pos == std::string::npos) return {};
                    auto end = json.find(']', pos);
                    if (end == std::string::npos) return {};
                    std::string arr = json.substr(pos + 1, end - pos - 1);
                    std::vector<double> res;
                    std::istringstream ss(arr);
                    std::string token;
                    while (std::getline(ss, token, ',')) {
                        try { res.push_back(std::stod(token)); }
                        catch (...) {}
                    }
                    return res;
                };
                auto find_id = [&]() -> int {
                    auto pos = json.find("\"id\"");
                    if (pos == std::string::npos) return -1;
                    pos = json.find(':', pos + 3);
                    if (pos == std::string::npos) return -1;
                    auto end = json.find_first_of(",}", pos);
                    if (end == std::string::npos) return -1;
                    try { return std::stoi(json.substr(pos + 1, end - pos - 1)); }
                    catch (...) { return -1; }
                };

                std::string cmd = find_str("cmd");
                auto args = find_arr("args");
                int req_id = find_id();

                std::vector<double> result;
                bool success = false;
                if (server->dispatch_ && !cmd.empty()) {
                    success = server->dispatch_(cmd, args, result);
                }

                std::string reply = "{\"id\":" + std::to_string(req_id) +
                                    ",\"success\":" + (success ? "true" : "false") +
                                    ",\"result\":[";
                for (size_t i = 0; i < result.size(); ++i) {
                    if (i) reply += ",";
                    reply += std::to_string(result[i]);
                }
                reply += "]}";

                unsigned char buf[LWS_PRE + 4096];
                int n = reply.copy(reinterpret_cast<char*>(buf) + LWS_PRE, 4096);
                lws_write(wsi, buf + LWS_PRE, n, LWS_WRITE_TEXT);
            }
            break;
        }

        case LWS_CALLBACK_SERVER_WRITEABLE: {
            // 每次 WRITEABLE 都发送当前最新状态
            std::string json;
            {
                std::lock_guard lock(server->state_mutex_);
                json = server->last_state_json_;
            }
            if (!json.empty()) {
                unsigned char buf[LWS_PRE + 8192];
                int n = json.copy(reinterpret_cast<char*>(buf) + LWS_PRE, 8192);
                lws_write(wsi, buf + LWS_PRE, n, LWS_WRITE_TEXT);
            }
            break;
        }

        default:
            break;
        }
        return 0;
    }

}  // namespace RusDriverNode
