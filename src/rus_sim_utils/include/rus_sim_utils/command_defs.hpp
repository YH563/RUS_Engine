#pragma once

// ════════════════════════════════════════════════════════════════════
//  字符串常量定义（协议域）
//  ────────────────────────────────────────────────────────────────────
//  仅字符串常量，不包含任何类型 / 逻辑。
//  指令名统一平铺在 CmdName 下，不区分高层/底层/模块——
//  "指令归属哪个模块、算什么层级"是路由域的配置（command_registry.hpp），
//  不属于协议层关注的事。
//  依赖：无
// ════════════════════════════════════════════════════════════════════

#include <string_view>

namespace RusUtils {

    // ── WebSocket 通道路径（前端连接路径） ──
    namespace WsPath {
        inline constexpr std::string_view kControl = "/control";  // command / reply / event
        inline constexpr std::string_view kState   = "/state";    // state 高频流（可丢帧）
        inline constexpr std::string_view kSensor  = "/sensor";   // 感知二进制帧（可丢帧，预留）
    }

    // ── 指令名（统一平铺） ──
    namespace CmdName {
        inline constexpr std::string_view kConnect          = "connect";
        inline constexpr std::string_view kShutdown         = "shutdown";
        // 模式切换（bridge 本地处理，修改扇出目标）：0=手动（直控 driver），1=自动（planning 协调）
        inline constexpr std::string_view kSetMode          = "set_mode";
        inline constexpr std::string_view kPreScanStart     = "pre_scan_start";
        inline constexpr std::string_view kPreScanEnd       = "pre_scan_end";
        inline constexpr std::string_view kSetStartPose     = "set_start_pose";
        inline constexpr std::string_view kSetEndPose       = "set_end_pose";
        inline constexpr std::string_view kPlan             = "plan";
        inline constexpr std::string_view kExecute          = "execute";
        inline constexpr std::string_view kStop             = "stop";
        inline constexpr std::string_view kPause            = "pause";
        inline constexpr std::string_view kResume           = "resume";
        inline constexpr std::string_view kReset            = "reset";
        inline constexpr std::string_view kQueryPreScanDone = "query_prescan_done";
        inline constexpr std::string_view kQueryMotionDone  = "query_motion_done";
        inline constexpr std::string_view kMapClear         = "map_clear";
        inline constexpr std::string_view kLoadCloud        = "load_cloud";
        inline constexpr std::string_view kPreScanDone      = "pre_scan_done";

        inline constexpr std::string_view kMoveJ            = "movej";
        inline constexpr std::string_view kMoveL            = "movel";
        inline constexpr std::string_view kServoJ           = "servoj";
        inline constexpr std::string_view kServoCart        = "servo_cart";
        inline constexpr std::string_view kStartJog         = "start_jog";
        inline constexpr std::string_view kStopJogDecel     = "stop_jog_decel";
        inline constexpr std::string_view kStopJogImmediate = "stop_jog_immediate";
        inline constexpr std::string_view kServoStart       = "servo_start";
        inline constexpr std::string_view kServoEnd         = "servo_end";
        inline constexpr std::string_view kDisconnect       = "disconnect";
        inline constexpr std::string_view kIsConnected      = "is_connected";
        inline constexpr std::string_view kIsInDragTeach    = "is_in_drag_teach";
        inline constexpr std::string_view kRobotEnable      = "robot_enable";
        inline constexpr std::string_view kGetState         = "get_state";
        inline constexpr std::string_view kIsMotionDone     = "is_motion_done";
        inline constexpr std::string_view kRunFile          = "run_file";
        inline constexpr std::string_view kSwitchDriver     = "switch_driver";
        inline constexpr std::string_view kSetTimeSpeed     = "set_time_speed";
        inline constexpr std::string_view kGetTimeSpeed     = "get_time_speed";
        inline constexpr std::string_view kGetSimTime       = "get_sim_time";
        inline constexpr std::string_view kStepOnce         = "step_once";
        inline constexpr std::string_view kGetFrameRate     = "get_frame_rate";

        // ── 工具坐标系 / 标定 ──
        inline constexpr std::string_view kSetToolCalibPoint = "set_tool_calib_point";
        inline constexpr std::string_view kComputeToolCalib  = "compute_tool_calib";
        inline constexpr std::string_view kSetToolCoord      = "set_tool_coord";
        inline constexpr std::string_view kSetToolIndex      = "set_tool_index";
        inline constexpr std::string_view kGetToolCoords     = "get_tool_coords";

        // 感知控制指令：通道已预留（WsPath::kSensor），指令待后续设计，暂不定义
    }

    // ── 事件名（ResultMessage::event 取值 / ModuleEvent.event） ──
    namespace EventName {
        inline constexpr std::string_view kPreScanDone = "pre_scan_done";
        inline constexpr std::string_view kPlanDone    = "plan_done";
        inline constexpr std::string_view kScanDone    = "scan_done";
        inline constexpr std::string_view kMotionDone  = "motion_done";
        inline constexpr std::string_view kError       = "error";
    }

    // ── 感知帧类型（SensorFrame::type；通道保留，具体内容待设计） ──
    namespace SensorType {
        inline constexpr std::string_view kPointCloud = "pointcloud";
        inline constexpr std::string_view kImage      = "image";
        inline constexpr std::string_view kCompressed = "compressed";
    }

}  // namespace RusUtils
