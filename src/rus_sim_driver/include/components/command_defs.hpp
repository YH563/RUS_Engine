#pragma once

#include <string_view>

namespace RusRobotDriver {

    /**
    * @brief 指令字符串常量
    *
    * 所有指令在此统一定义，避免魔数字符串散落在各处。
    * 用于 CmdParser 解析结果匹配、DriverNode::dispatch 分发等场景。
    */
    namespace Cmd {

        // ── 运动指令 ──
        inline constexpr std::string_view kMoveJ      = "movej";
        inline constexpr std::string_view kMoveL      = "movel";
        inline constexpr std::string_view kServoJ     = "servoj";
        inline constexpr std::string_view kServoCart  = "servo_cart";
        inline constexpr std::string_view kStartJog       = "start_jog";
        inline constexpr std::string_view kStopJOGDecel    = "stop_jog_decel";
        inline constexpr std::string_view kStopJOGImmediate = "stop_jog_immediate";

        // ── 伺服模式 ──
        inline constexpr std::string_view kServoStart = "servo_start";
        inline constexpr std::string_view kServoEnd   = "servo_end";

        // ── 运动控制 ──
        inline constexpr std::string_view kStop       = "stop";
        inline constexpr std::string_view kPause      = "pause";
        inline constexpr std::string_view kResume     = "resume";

        // ── 驱动控制 ──
        inline constexpr std::string_view kConnect       = "connect";
        inline constexpr std::string_view kDisconnect    = "disconnect";
        inline constexpr std::string_view kIsConnected   = "is_connected";
        inline constexpr std::string_view kIsInDragTeach = "is_in_drag_teach";
        inline constexpr std::string_view kRobotEnable   = "robot_enable";
        inline constexpr std::string_view kGetState      = "get_state";
        inline constexpr std::string_view kIsMotionDone  = "is_motion_done";

        // ── 录制 / 回放 ──
        inline constexpr std::string_view kRecordStart   = "record_start";
        inline constexpr std::string_view kRecordStop    = "record_stop";
        inline constexpr std::string_view kPlaybackStart = "playback_start";
        inline constexpr std::string_view kPlaybackStop  = "playback_stop";
        inline constexpr std::string_view kPlaybackPause      = "playback_pause";
        inline constexpr std::string_view kPlaybackResume     = "playback_resume";
        inline constexpr std::string_view kPlaybackSetSpeed   = "playback_set_speed";
        inline constexpr std::string_view kPlaybackSeek       = "playback_seek";
        inline constexpr std::string_view kPlaybackStep       = "playback_step";
        inline constexpr std::string_view kPlaybackSetLoop    = "playback_set_loop";
        inline constexpr std::string_view kPlaybackGetInfo    = "playback_get_info";

        // ── 文件执行 ──
        inline constexpr std::string_view kRunFile       = "run_file";

        // ── 驱动切换 ──
        inline constexpr std::string_view kSwitchDriver   = "switch_driver";

        // ── 仿真控制（仅 Sim 驱动） ──
        inline constexpr std::string_view kSetTimeSpeed    = "set_time_speed";
        inline constexpr std::string_view kGetTimeSpeed    = "get_time_speed";
        inline constexpr std::string_view kGetSimTime      = "get_sim_time";
        inline constexpr std::string_view kStepOnce        = "step_once";
        inline constexpr std::string_view kIsPlaybackActive = "is_playback_active";
        inline constexpr std::string_view kGetFrameRate     = "get_frame_rate";

    }  // namespace Cmd
    
}  // namespace RusRobotDriver
