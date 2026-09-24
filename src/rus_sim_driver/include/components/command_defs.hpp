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
        inline constexpr std::string_view kReset      = "reset";

        // ── 驱动控制 ──
        inline constexpr std::string_view kConnect       = "connect";
        inline constexpr std::string_view kDisconnect    = "disconnect";
        inline constexpr std::string_view kIsConnected   = "is_connected";
        inline constexpr std::string_view kIsInDragTeach = "is_in_drag_teach";
        inline constexpr std::string_view kRobotEnable   = "robot_enable";
        inline constexpr std::string_view kGetState      = "get_state";
        inline constexpr std::string_view kIsMotionDone  = "is_motion_done";
        // 扇出指令：bridge 会同时发给 planning + driver，驱动侧与 is_motion_done 等价
        inline constexpr std::string_view kQueryMotionDone = "query_motion_done";

        // ── 文件执行 ──
        inline constexpr std::string_view kRunFile       = "run_file";

        // ── 驱动切换 ──
        inline constexpr std::string_view kSwitchDriver   = "switch_driver";
        // 查询当前驱动类型：0=仿真（sim），1=真实（real），与 switch_driver 的 type 参数同编码
        inline constexpr std::string_view kGetDriverType  = "get_driver_type";

        // ── 仿真控制（仅 Sim 驱动） ──
        inline constexpr std::string_view kSetTimeSpeed    = "set_time_speed";
        inline constexpr std::string_view kGetTimeSpeed    = "get_time_speed";
        inline constexpr std::string_view kGetSimTime      = "get_sim_time";
        inline constexpr std::string_view kStepOnce        = "step_once";
        inline constexpr std::string_view kGetFrameRate     = "get_frame_rate";

        // ── 工具坐标系 / 标定 ──
        inline constexpr std::string_view kSetToolCalibPoint = "set_tool_calib_point";
        inline constexpr std::string_view kComputeToolCalib  = "compute_tool_calib";
        inline constexpr std::string_view kSetToolCoord      = "set_tool_coord";
        inline constexpr std::string_view kSetToolIndex      = "set_tool_index";
        inline constexpr std::string_view kGetToolCoords     = "get_tool_coords";

    }  // namespace Cmd
    
}  // namespace RusRobotDriver
