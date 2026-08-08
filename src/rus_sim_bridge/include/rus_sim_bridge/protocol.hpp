#pragma once

#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace rus_sim_bridge {

    // ════════════════════════════════════════════════════════════════
    //  通道定义（同一端口多连接，按路径区分）
    // ════════════════════════════════════════════════════════════════
    enum class Channel {
        Control,   // /ws/control  command / reply / event（可靠）
        State,     // /ws/state    state 高频流（可丢帧）
        Sensor,    // /ws/sensor   图像 / 点云（预留）
    };

    inline const char* channel_path(Channel c) {
        switch (c) {
            case Channel::Control: return "/control";
            case Channel::State:   return "/state";
            case Channel::Sensor:  return "/sensor";
        }
        return "/control";
    }

    // ════════════════════════════════════════════════════════════════
    //  通路 A：前端 → 后端（统一 command）
    // ════════════════════════════════════════════════════════════════
    struct CommandMessage {
        uint64_t id = 0;              // 客户端自增，用于关联 reply
        std::string cmd;              // 指令名（见 Cmd 命名空间）
        std::vector<double> args;     // 参数数组（可为空）
    };

    // ════════════════════════════════════════════════════════════════
    //  通路 B：后端 → 前端
    // ════════════════════════════════════════════════════════════════
    struct ReplyMessage {
        uint64_t id = 0;              // 与请求关联
        bool success = false;
        std::string message;          // "ok" / 错误描述
        std::vector<double> result;   // 查询类带数据，操作类为空
    };

    struct EventMessage {
        std::string event;            // 事件名（如 "scan_done" / "error"）
        std::string data;             // 可选结构化数据（JSON 字符串，可为空）
    };

    struct StateMessage {
        double timestamp = 0.0;
        double frame_rate = 0.0;
        std::vector<double> joint_pos;
        std::vector<double> joint_vel;
        std::vector<double> joint_acc;
        std::vector<double> effort;
        std::vector<double> flange_pos;
    };

    // ════════════════════════════════════════════════════════════════
    //  指令常量（三层合并，前端唯一入口）
    //  前缀约定：高=高层意图，底=底层驱动，数=数据模块
    // ════════════════════════════════════════════════════════════════
    namespace Cmd {

        // ── 高层意图 ──
        inline constexpr std::string_view kConnect       = "connect";
        inline constexpr std::string_view kShutdown      = "shutdown";
        inline constexpr std::string_view kPreScanStart  = "pre_scan_start";
        inline constexpr std::string_view kPreScanEnd    = "pre_scan_end";
        inline constexpr std::string_view kSetStartPose  = "set_start_pose";
        inline constexpr std::string_view kSetEndPose    = "set_end_pose";
        inline constexpr std::string_view kPlan          = "plan";
        inline constexpr std::string_view kExecute       = "execute";
        inline constexpr std::string_view kStop          = "stop";
        inline constexpr std::string_view kPause         = "pause";
        inline constexpr std::string_view kResume        = "resume";
        inline constexpr std::string_view kReset         = "reset";
        inline constexpr std::string_view kQueryPreScanDone = "query_prescan_done";
        inline constexpr std::string_view kQueryMotionDone  = "query_motion_done";

        // ── 底层驱动（透传，高级/调试模式） ──
        inline constexpr std::string_view kMoveJ        = "movej";
        inline constexpr std::string_view kMoveL        = "movel";
        inline constexpr std::string_view kServoJ       = "servoj";
        inline constexpr std::string_view kServoCart    = "servo_cart";
        inline constexpr std::string_view kStartJog     = "start_jog";
        inline constexpr std::string_view kStopJOGDecel = "stop_jog_decel";
        inline constexpr std::string_view kStopJOGImmediate = "stop_jog_immediate";
        inline constexpr std::string_view kServoStart   = "servo_start";
        inline constexpr std::string_view kServoEnd     = "servo_end";
        inline constexpr std::string_view kDisconnect   = "disconnect";
        inline constexpr std::string_view kIsConnected  = "is_connected";
        inline constexpr std::string_view kIsInDragTeach = "is_in_drag_teach";
        inline constexpr std::string_view kRobotEnable  = "robot_enable";
        inline constexpr std::string_view kGetState     = "get_state";
        inline constexpr std::string_view kIsMotionDone = "is_motion_done";
        inline constexpr std::string_view kRunFile      = "run_file";
        inline constexpr std::string_view kSwitchDriver = "switch_driver";
        inline constexpr std::string_view kSetTimeSpeed = "set_time_speed";
        inline constexpr std::string_view kGetTimeSpeed = "get_time_speed";
        inline constexpr std::string_view kGetSimTime   = "get_sim_time";
        inline constexpr std::string_view kStepOnce     = "step_once";
        inline constexpr std::string_view kGetFrameRate = "get_frame_rate";

        // ── 数据模块（录制 / 回放） ──
        inline constexpr std::string_view kRecordStart     = "record_start";
        inline constexpr std::string_view kRecordStop      = "record_stop";
        inline constexpr std::string_view kPlaybackStart   = "playback_start";
        inline constexpr std::string_view kPlaybackStop    = "playback_stop";
        inline constexpr std::string_view kPlaybackPause   = "playback_pause";
        inline constexpr std::string_view kPlaybackResume  = "playback_resume";
        inline constexpr std::string_view kPlaybackSetSpeed = "playback_set_speed";
        inline constexpr std::string_view kPlaybackSeek    = "playback_seek";
        inline constexpr std::string_view kPlaybackStep    = "playback_step";
        inline constexpr std::string_view kPlaybackSetLoop = "playback_set_loop";
        inline constexpr std::string_view kPlaybackGetInfo = "playback_get_info";

    }  // namespace Cmd

    // ════════════════════════════════════════════════════════════════
    //  JSON 编解码（轻量手写，无第三方依赖）
    // ════════════════════════════════════════════════════════════════

    /// 前端 JSON → CommandMessage（只支持本协议子集）
    /// 兼容：{"type":"command","id":1,"cmd":"movej","args":[...]}
    /// 不要求 type 字段（后端默认收到的都是 command）
    bool parse_command(const std::string& json, CommandMessage& out);

    /// 生成 reply JSON
    std::string serialize_reply(const ReplyMessage& r);

    /// 生成 event JSON
    std::string serialize_event(const EventMessage& e);

    /// 生成 state JSON
    std::string serialize_state(const StateMessage& s);

}  // namespace rus_sim_bridge
