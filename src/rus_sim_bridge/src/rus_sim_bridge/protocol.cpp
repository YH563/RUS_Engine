#include "rus_sim_bridge/protocol.hpp"

#include <cstdio>
#include <sstream>

namespace rus_sim_bridge {

    // ============================================================
    //  内部小工具（手写 JSON，无第三方依赖）
    // ============================================================

    namespace {

        std::string dtoa(double v) {
            char buf[40];
            snprintf(buf, sizeof(buf), "%.6f", v);
            return buf;
        }

        std::string arr_to_json(const std::vector<double>& v) {
            std::string s = "[";
            for (size_t i = 0; i < v.size(); ++i) {
                if (i) s += ",";
                s += dtoa(v[i]);
            }
            s += "]";
            return s;
        }

        std::string escape(const std::string& in) {
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

        std::string find_str(const std::string& json, const std::string& key) {
            auto pos = json.find("\"" + key + "\"");
            if (pos == std::string::npos) return "";
            pos = json.find('"', pos + key.size() + 3);
            if (pos == std::string::npos) return "";
            auto end = json.find('"', pos + 1);
            if (end == std::string::npos) return "";
            return json.substr(pos + 1, end - pos - 1);
        }

        std::vector<double> find_arr(const std::string& json, const std::string& key) {
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
                while (!token.empty() && (token.front() == ' ' || token.front() == '\t')) token.erase(token.begin());
                while (!token.empty() && (token.back() == ' ' || token.back() == '\t')) token.pop_back();
                if (token.empty()) continue;
                try { res.push_back(std::stod(token)); } catch (...) {}
            }
            return res;
        }

        long long find_id(const std::string& json) {
            auto pos = json.find("\"id\"");
            if (pos == std::string::npos) return -1;
            pos = json.find(':', pos + 3);
            if (pos == std::string::npos) return -1;
            auto end = json.find_first_of(",}", pos);
            if (end == std::string::npos) return -1;
            try { return std::stoll(json.substr(pos + 1, end - pos - 1)); }
            catch (...) { return -1; }
        }

    }  // namespace

    // ============================================================
    //  通路 A：前端 JSON → CommandMessage
    // ============================================================

    bool parse_command(const std::string& json, CommandMessage& out) {
        std::string cmd = find_str(json, "cmd");
        if (cmd.empty()) return false;
        out.cmd = std::move(cmd);
        out.args = find_arr(json, "args");
        long long id = find_id(json);
        out.id = (id < 0) ? 0 : static_cast<uint64_t>(id);
        return true;
    }

    // ============================================================
    //  通路 B：后端 → 前端 JSON
    // ============================================================

    std::string serialize_reply(const ReplyMessage& r) {
        return std::string("{\"type\":\"reply\",\"id\":") + std::to_string(r.id) +
               ",\"success\":" + (r.success ? "true" : "false") +
               ",\"message\":\"" + escape(r.message) + "\"" +
               ",\"result\":" + arr_to_json(r.result) + "}";
    }

    std::string serialize_event(const EventMessage& e) {
        std::string data = e.data.empty() ? "{}" : e.data;
        return std::string("{\"type\":\"event\",\"event\":\"") + escape(e.event) +
               "\",\"data\":" + data + "}";
    }

    std::string serialize_state(const StateMessage& s) {
        return std::string("{\"type\":\"state\",\"timestamp\":") + dtoa(s.timestamp) +
               ",\"frame_rate\":" + dtoa(s.frame_rate) +
               ",\"joint_pos\":" + arr_to_json(s.joint_pos) +
               ",\"joint_vel\":" + arr_to_json(s.joint_vel) +
               ",\"joint_acc\":" + arr_to_json(s.joint_acc) +
               ",\"effort\":" + arr_to_json(s.effort) +
               ",\"flange_pos\":" + arr_to_json(s.flange_pos) + "}";
    }

}  // namespace rus_sim_bridge
