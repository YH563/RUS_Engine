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
        // 查询当前驱动类型：0=仿真（sim），1=真实（real），与 switch_driver 的 type 参数同编码
        inline constexpr std::string_view kGetDriverType    = "get_driver_type";
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

        // ── 回放（rus_sim_recorder_replay：按 .rusrec 时间轴把录音重发回话题）──
        //   时间轴口径 / 状态编码见 docs/Protocol/WsProtocol.md §4.7
        inline constexpr std::string_view kReplayLoad     = "replay_load";
        inline constexpr std::string_view kReplayList     = "replay_list";
        inline constexpr std::string_view kReplayStart    = "replay_start";
        inline constexpr std::string_view kReplayPause    = "replay_pause";
        inline constexpr std::string_view kReplayResume   = "replay_resume";
        inline constexpr std::string_view kReplayStop     = "replay_stop";
        inline constexpr std::string_view kReplaySeek     = "replay_seek";
        inline constexpr std::string_view kReplaySetSpeed = "replay_set_speed";
        inline constexpr std::string_view kReplayStep     = "replay_step";
        inline constexpr std::string_view kReplayStatus   = "replay_status";

        // ── 录制控制（recorder_node：外部控制开始 / 结束 / 查询落盘状态）──
        //   节点默认启动即录（autostart=true），这三条用于运行期开关录制；
        //   状态编码 / result 字段见 docs/Protocol/WsProtocol.md §4.8
        inline constexpr std::string_view kRecorderStart  = "recorder_start";
        inline constexpr std::string_view kRecorderStop   = "recorder_stop";
        inline constexpr std::string_view kRecorderStatus = "recorder_status";

        // 感知控制指令：通道已预留（WsPath::kSensor），指令待后续设计，暂不定义
    }

    // ── 事件名（ResultMessage::event 取值 / ModuleEvent.event） ──
    namespace EventName {
        inline constexpr std::string_view kPreScanDone = "pre_scan_done";
        inline constexpr std::string_view kPlanDone    = "plan_done";
        inline constexpr std::string_view kScanDone    = "scan_done";
        inline constexpr std::string_view kMotionDone  = "motion_done";
        inline constexpr std::string_view kReplayDone  = "replay_done";  // 回放播到末尾（非 loop）
        inline constexpr std::string_view kError       = "error";
    }

    // ── 感知帧类型（SensorFrame::type；与 rus_sim_interfaces/msg/SensorFrame.msg 的 TYPE_* 对应） ──
    namespace SensorType {
        inline constexpr std::string_view kPointCloud = "pointcloud";
        inline constexpr std::string_view kImage      = "image";
        inline constexpr std::string_view kUltrasound = "ultrasound";  // 预留：超声（尚未实现）
        inline constexpr std::string_view kCompressed = "compressed";
    }

    // ── 感知帧数据语义（SensorFrame::scope；前端据此区分"当前帧"与"累积地图"） ──
    namespace SensorScope {
        inline constexpr std::string_view kFrame = "frame";  // 单视角当前帧（/perception/frame 同源）
        inline constexpr std::string_view kMap   = "map";    // 累积地图快照（rolling/accumulate 快照）
    }

}  // namespace RusUtils
