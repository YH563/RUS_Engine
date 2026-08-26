#pragma once

// ════════════════════════════════════════════════════════════════════
//  统一协议定义（纯协议层）
//  ────────────────────────────────────────────────────────────────────
//  只包含：通道 / 通路 / 消息结构 / JSON 编解码 / 感知二进制帧。
//  不含指令定义（见 command_types.hpp）与路由（见 command_registry.hpp）。
//
//  设计原则（详见 docs/ws_protocol.md）：
//    1. 两条单向通路：通路 A = command（前端→后端），通路 B = reply/event/state/sensor
//    2. reply / event 同构（success / message / result），事件用 ack_id 关联原指令
//    3. 状态流（state）覆盖式可丢帧；感知流（sensor）为独立二进制通道
//  依赖：command_defs.hpp（通道路径 / 事件名 / 帧类型常量）
// ════════════════════════════════════════════════════════════════════

#include <cstdint>
#include <cstdio>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "command_defs.hpp"

namespace RusUtils {

    // ════════════════════════════════════════════════════════════════
    //  通道定义（同一端口多连接，按路径区分）
    // ════════════════════════════════════════════════════════════════
    enum class Channel {
        Control,   // /control  command / reply / event（可靠）
        State,     // /state    state 高频流（可丢帧）
        Sensor,    // /sensor   感知二进制帧（可丢帧）
    };

    /// 通道名（lws 子协议名 / 前端连接路径）
    constexpr std::string_view channel_path(Channel c) {
        switch (c) {
            case Channel::Control: return WsPath::kControl;
            case Channel::State:   return WsPath::kState;
            case Channel::Sensor:  return WsPath::kSensor;
        }
        return WsPath::kControl;
    }

    // ════════════════════════════════════════════════════════════════
    //  通路 A：前端 → 后端（统一 command）
    // ════════════════════════════════════════════════════════════════
    struct CommandMessage {
        uint32_t id = 0;              // 客户端自增，用于关联 reply
        std::string cmd;              // 指令名（与 Cmd 结构体 kName 一致）
        std::vector<double> args;     // 参数数组（可为空）
    };

    // ════════════════════════════════════════════════════════════════
    //  通路 B：统一回执（reply / event 同构）
    //  ────────────────────────────────────────────────────────────────
    //  reply —— 对 command 的同步应答（查询结果 / 校验失败 / 已受理）
    //  event —— 子模块的异步触发 / 完成通知（ack_id 关联原指令）
    // ════════════════════════════════════════════════════════════════
    struct ResultMessage {
        enum class Kind { Reply, Event };

        Kind kind = Kind::Reply;

        uint32_t id = 0;              // reply：关联 command id；event：固定 0
        uint32_t ack_id = 0;          // event：触发它的 command id（0 = 无关联）
        std::string event;            // 事件名（仅 Event，见 EventName 命名空间）
        bool success = false;         // 是否成功
        std::string message;          // 错误描述 / 附加说明（成功可为空）
        std::vector<double> result;   // 处理结果（查询类带数据，操作类为空）

        static ResultMessage MakeReply(uint32_t id, bool ok,
                                       std::string msg,
                                       std::vector<double> res = {}) {
            ResultMessage r;
            r.kind = Kind::Reply;
            r.id = id;
            r.success = ok;
            r.message = std::move(msg);
            r.result = std::move(res);
            return r;
        }

        static ResultMessage MakeEvent(std::string evt, uint32_t ack_id, bool ok,
                                       std::string msg = {},
                                       std::vector<double> res = {}) {
            ResultMessage r;
            r.kind = Kind::Event;
            r.event = std::move(evt);
            r.ack_id = ack_id;
            r.success = ok;
            r.message = std::move(msg);
            r.result = std::move(res);
            return r;
        }
    };

    // ════════════════════════════════════════════════════════════════
    //  通路 B：状态流（/state 通道，可丢帧）
    // ════════════════════════════════════════════════════════════════
    struct StateMessage {
        double timestamp = 0.0;
        double frame_rate = 0.0;
        std::vector<double> joint_pos;
        std::vector<double> joint_vel;
        std::vector<double> joint_acc;
        std::vector<double> effort;
        std::vector<double> flange_pos;
        int tool_index = 0;          // 当前工具坐标系索引（0~14）
        std::vector<double> tool_pose;  // 当前 TCP 位姿（基坐标系下）XYZABC [m/rad]
    };

    // ════════════════════════════════════════════════════════════════
    //  通路 B：感知数据帧（/sensor 二进制通道）
    //  ────────────────────────────────────────────────────────────────
    //  线格式：uint32 LE 头长度 + JSON 头 + 二进制 payload
    // ════════════════════════════════════════════════════════════════
    struct SensorFrame {
        std::string type;             // 帧类型（见 SensorType 命名空间）
        double timestamp = 0.0;
        uint32_t seq = 0;             // 帧序号（前端检测丢帧）

        // ── pointcloud 元数据 ──
        uint32_t points = 0;          // 点数
        std::vector<std::string> fields;  // 分量顺序：如 x,y,z,intensity
        std::string dtype;            // float32 / float64 / uint8 / int32 ...

        // ── image 元数据 ──
        uint32_t width = 0;
        uint32_t height = 0;
        std::string encoding;         // rgb8 / bgr8 / mono8 ...
        uint32_t step = 0;            // 每行字节数（含 padding）

        // ── payload ──
        std::vector<uint8_t> payload;
    };

    // ════════════════════════════════════════════════════════════════
    //  JSON 编解码（轻量手写，无第三方依赖）
    // ════════════════════════════════════════════════════════════════

    namespace detail {

        inline std::string json_escape(const std::string& in) {
            std::string out;
            out.reserve(in.size());
            for (char c : in) {
                switch (c) {
                    case '"':  out += "\\\""; break;
                    case '\\': out += "\\\\"; break;
                    case '\n': out += "\\n";  break;
                    case '\r': out += "\\r";  break;
                    case '\t': out += "\\t";  break;
                    default:   out += c;      break;
                }
            }
            return out;
        }

        /// 保留 6 位小数（协议 v0.2；如需更高精度后续调整）
        inline std::string dtoa(double v) {
            char buf[40];
            snprintf(buf, sizeof(buf), "%.6f", v);
            return buf;
        }

        inline std::string arr_to_json(const std::vector<double>& v) {
            std::string s = "[";
            for (size_t i = 0; i < v.size(); ++i) {
                if (i) s += ",";
                s += dtoa(v[i]);
            }
            s += "]";
            return s;
        }

        /// 在 JSON 中提取 "key" : "value" 的字符串值（跳过空白，处理转义引号）
        inline std::string find_str(const std::string& json, const std::string& key) {
            auto pos = json.find("\"" + key + "\"");
            if (pos == std::string::npos) return "";
            pos = json.find(':', pos + key.size() + 2);
            if (pos == std::string::npos) return "";
            while (pos + 1 < json.size() && (json[pos + 1] == ' ' || json[pos + 1] == '\t')) ++pos;
            if (pos + 1 >= json.size() || json[pos + 1] != '"') return "";
            ++pos;  // 定位到开头引号
            ++pos;
            std::string out;
            while (pos < json.size() && json[pos] != '"') {
                if (json[pos] == '\\' && pos + 1 < json.size()) {
                    out += json[pos + 1];
                    pos += 2;
                    continue;
                }
                out += json[pos++];
            }
            return out;
        }

        /// 提取 "key" : [...] 的 double 数组
        inline std::vector<double> find_arr(const std::string& json, const std::string& key) {
            auto pos = json.find("\"" + key + "\"");
            if (pos == std::string::npos) return {};
            pos = json.find('[', pos + key.size() + 2);
            if (pos == std::string::npos) return {};
            auto end = json.find(']', pos);
            if (end == std::string::npos) return {};
            std::string arr = json.substr(pos + 1, end - pos - 1);
            std::vector<double> res;
            std::istringstream ss(arr);
            std::string token;
            while (std::getline(ss, token, ',')) {
                while (!token.empty() && (token.front() == ' ' || token.front() == '\t')) token.erase(token.begin());
                while (!token.empty() && (token.back() == ' ' || token.back() == '\t')) token.pop_back();
                if (token.empty()) continue;
                try { res.push_back(std::stod(token)); } catch (...) {}
            }
            return res;
        }

        inline uint32_t find_id(const std::string& json) {
            auto pos = json.find("\"id\"");
            if (pos == std::string::npos) return 0;
            pos = json.find(':', pos + 4);
            if (pos == std::string::npos) return 0;
            auto end = json.find_first_of(",}", pos);
            if (end == std::string::npos) return 0;
            try {
                return static_cast<uint32_t>(std::stoull(json.substr(pos + 1, end - pos - 1)));
            } catch (...) {
                return 0;
            }
        }

        inline void append_u32_le(std::vector<uint8_t>& out, uint32_t v) {
            out.push_back(static_cast<uint8_t>(v & 0xFF));
            out.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
            out.push_back(static_cast<uint8_t>((v >> 16) & 0xFF));
            out.push_back(static_cast<uint8_t>((v >> 24) & 0xFF));
        }

        inline uint32_t read_u32_le(const uint8_t* p) {
            return static_cast<uint32_t>(p[0])
                 | (static_cast<uint32_t>(p[1]) << 8)
                 | (static_cast<uint32_t>(p[2]) << 16)
                 | (static_cast<uint32_t>(p[3]) << 24);
        }

    }  // namespace detail

    /// 前端 JSON → CommandMessage
    inline bool ParseCommandMessage(const std::string& json, CommandMessage& out) {
        std::string cmd = detail::find_str(json, "cmd");
        if (cmd.empty()) return false;
        out.cmd = std::move(cmd);
        out.args = detail::find_arr(json, "args");
        out.id = detail::find_id(json);
        return true;
    }

    /// 统一回执 → JSON（reply / event 同构）
    inline std::string SerializeResult(const ResultMessage& r) {
        std::string s;
        if (r.kind == ResultMessage::Kind::Event) {
            s = "{\"type\":\"event\",\"id\":0,\"ack_id\":" + std::to_string(r.ack_id) +
                ",\"event\":\"" + detail::json_escape(r.event) + "\"";
        } else {
            s = "{\"type\":\"reply\",\"id\":" + std::to_string(r.id);
        }
        s += ",\"success\":" + std::string(r.success ? "true" : "false") +
             ",\"message\":\"" + detail::json_escape(r.message) + "\"" +
             ",\"result\":" + detail::arr_to_json(r.result) + "}";
        return s;
    }

    /// 状态流 → JSON
    inline std::string SerializeState(const StateMessage& s) {
        return std::string("{\"type\":\"state\",\"timestamp\":") + detail::dtoa(s.timestamp) +
               ",\"frame_rate\":" + detail::dtoa(s.frame_rate) +
               ",\"joint_pos\":" + detail::arr_to_json(s.joint_pos) +
               ",\"joint_vel\":" + detail::arr_to_json(s.joint_vel) +
               ",\"joint_acc\":" + detail::arr_to_json(s.joint_acc) +
               ",\"effort\":" + detail::arr_to_json(s.effort) +
               ",\"flange_pos\":" + detail::arr_to_json(s.flange_pos) +
               ",\"tool_index\":" + std::to_string(s.tool_index) +
               ",\"tool_pose\":" + detail::arr_to_json(s.tool_pose) + "}";
    }

    /// 编码感知帧 → 二进制（uint32 LE 头长度 + JSON 头 + payload）
    inline std::vector<uint8_t> EncodeSensorFrame(const SensorFrame& f) {
        std::string head = "{\"type\":\"" + detail::json_escape(f.type) + "\"";

        std::string fields_json = "[";
        for (size_t i = 0; i < f.fields.size(); ++i) {
            if (i) fields_json += ",";
            fields_json += "\"" + detail::json_escape(f.fields[i]) + "\"";
        }
        fields_json += "]";

        if (f.type == SensorType::kPointCloud) {
            head += ",\"points\":" + std::to_string(f.points) +
                    ",\"fields\":" + fields_json +
                    ",\"dtype\":\"" + detail::json_escape(f.dtype) + "\"";
        } else if (f.type == SensorType::kImage) {
            head += ",\"width\":" + std::to_string(f.width) +
                    ",\"height\":" + std::to_string(f.height) +
                    ",\"encoding\":\"" + detail::json_escape(f.encoding) + "\"" +
                    ",\"step\":" + std::to_string(f.step);
        }
        head += ",\"timestamp\":" + detail::dtoa(f.timestamp) +
                ",\"seq\":" + std::to_string(f.seq) + "}";

        std::vector<uint8_t> out;
        detail::append_u32_le(out, static_cast<uint32_t>(head.size()));
        out.insert(out.end(), head.begin(), head.end());
        out.insert(out.end(), f.payload.begin(), f.payload.end());
        return out;
    }

    /// 解码感知帧（成功返回 true；数据不足 / 头非法返回 false）
    inline bool DecodeSensorFrame(const std::vector<uint8_t>& buf, SensorFrame& out) {
        if (buf.size() < 4) return false;
        uint32_t head_len = detail::read_u32_le(buf.data());
        if (buf.size() < 4u + head_len) return false;
        std::string head(buf.begin() + 4, buf.begin() + 4 + head_len);

        out = SensorFrame{};
        out.type = detail::find_str(head, "type");

        auto num = [&](const std::string& k) -> double {
            auto pos = head.find("\"" + k + "\"");
            if (pos == std::string::npos) return 0.0;
            pos = head.find(':', pos + k.size() + 2);
            if (pos == std::string::npos) return 0.0;
            auto end = head.find_first_of(",}", pos);
            if (end == std::string::npos) return 0.0;
            try { return std::stod(head.substr(pos + 1, end - pos - 1)); }
            catch (...) { return 0.0; }
        };

        out.timestamp = num("timestamp");
        out.seq = static_cast<uint32_t>(num("seq"));

        if (out.type == SensorType::kPointCloud) {
            out.points = static_cast<uint32_t>(num("points"));
            out.dtype = detail::find_str(head, "dtype");
            auto p = head.find("\"fields\"");
            if (p != std::string::npos) {
                auto lb = head.find('[', p);
                auto rb = head.find(']', lb);
                if (lb != std::string::npos && rb != std::string::npos) {
                    std::string arr = head.substr(lb + 1, rb - lb - 1);
                    std::istringstream ss(arr);
                    std::string tok;
                    while (std::getline(ss, tok, ',')) {
                        while (!tok.empty() && (tok.front() == ' ' || tok.front() == '\t' || tok.front() == '"')) tok.erase(tok.begin());
                        while (!tok.empty() && (tok.back() == ' ' || tok.back() == '\t' || tok.back() == '"')) tok.pop_back();
                        if (!tok.empty()) out.fields.push_back(tok);
                    }
                }
            }
        } else if (out.type == SensorType::kImage) {
            out.width = static_cast<uint32_t>(num("width"));
            out.height = static_cast<uint32_t>(num("height"));
            out.encoding = detail::find_str(head, "encoding");
            out.step = static_cast<uint32_t>(num("step"));
        }

        out.payload.assign(buf.begin() + 4 + head_len, buf.end());
        return true;
    }

}  // namespace RusUtils
