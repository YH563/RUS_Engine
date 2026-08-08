#pragma once

#include <stdexcept>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

#include "command_definitions.hpp"

namespace RusUtils {

    // ── 高层指令类型（面向用户侧 → 后端） ──

    /** 查询：预扫查是否完成 */
    struct QueryPreScanDone {};

    /** 查询：运动是否完成 */
    struct QueryMotionDone {};

    /** 开始预扫查（点云建模） */
    struct PreScanStartCmd {};

    /** 设置扫查起点，args = [x, y, z] */
    struct SetStartPoseCmd {
        std::vector<double> pose;
    };

    /** 设置扫查终点，args = [x, y, z] */
    struct SetEndPoseCmd {
        std::vector<double> pose;
    };

    /** 生成扫查路径 */
    struct PlanCmd {};

    /** 执行扫查 */
    struct ExecuteCmd {};

    /** 停止所有运动（急停） */
    struct StopCmd {};

    /** 暂停扫查 */
    struct PauseCmd {};

    /** 继续扫查 */
    struct ResumeCmd {};

    /** 错误复位 */
    struct ResetCmd {};

    /** 连接驱动、上使能 */
    struct ConnectCmd {};

    /** 关闭系统 */
    struct ShutdownCmd {};

    /** 开始录制运动数据 */
    struct RecordStartCmd {};

    /** 停止录制 */
    struct RecordStopCmd {};

    /** 开始回放录制数据 */
    struct PlaybackStartCmd {};

    /** 停止回放 */
    struct PlaybackStopCmd {};

    /** 暂停回放 */
    struct PlaybackPauseCmd {};

    /** 继续回放 */
    struct PlaybackResumeCmd {};

    /** 设置回放速度倍率，args = [speed] */
    struct PlaybackSetSpeedCmd {
        std::vector<double> args;
    };

    /** 跳转回放时间，args = [time_seconds] */
    struct PlaybackSeekCmd {
        std::vector<double> args;
    };

    /** 逐帧步进，args = [direction] */
    struct PlaybackStepCmd {
        std::vector<double> args;
    };

    /** 设置循环播放，args = [enable] */
    struct PlaybackSetLoopCmd {
        std::vector<double> args;
    };

    /** 查询回放信息 */
    struct PlaybackGetInfoCmd {};

    // 统一指令承载
    using HighLevelCommand = std::variant<
        QueryMotionDone, QueryPreScanDone, PreScanStartCmd, SetStartPoseCmd, SetEndPoseCmd, PlanCmd, ExecuteCmd,
        StopCmd, PauseCmd, ResumeCmd, ResetCmd, ConnectCmd, ShutdownCmd,
        RecordStartCmd, RecordStopCmd,
        PlaybackStartCmd, PlaybackStopCmd, PlaybackPauseCmd, PlaybackResumeCmd,
        PlaybackSetSpeedCmd, PlaybackSeekCmd, PlaybackStepCmd, PlaybackSetLoopCmd, PlaybackGetInfoCmd>;


    // std::visit 辅助模板
    template<class... Ts> struct Overloaded : Ts... { using Ts::operator()...; };
    template<class... Ts> Overloaded(Ts...) -> Overloaded<Ts...>;

    /**
     * @brief 将指令名和参数数组解析为 HighLevelCommand
     *
     * @param name 指令名（见 command_definitions.hpp 的 Cmd 常量）
     * @param args 参数数组
     * @return HighLevelCommand
     * @throws std::invalid_argument 未知指令
     */
    inline HighLevelCommand ParseHighLevelCommand(
        std::string_view name, const std::vector<double>& args) {

        if (name == Cmd::kPreScanStart) return PreScanStartCmd{};
        if (name == Cmd::kSetStartPose) return SetStartPoseCmd{args};
        if (name == Cmd::kSetEndPose)   return SetEndPoseCmd{args};
        if (name == Cmd::kPlan)         return PlanCmd{};
        if (name == Cmd::kExecute)      return ExecuteCmd{};
        if (name == Cmd::kStop)         return StopCmd{};
        if (name == Cmd::kPause)        return PauseCmd{};
        if (name == Cmd::kResume)       return ResumeCmd{};
        if (name == Cmd::kReset)        return ResetCmd{};
        if (name == Cmd::kConnect)      return ConnectCmd{};
        if (name == Cmd::kShutdown)     return ShutdownCmd{};
        if (name == Cmd::kQueryPreScanDone) return QueryPreScanDone{};
        if (name == Cmd::kQueryMotionDone)  return QueryMotionDone{};

        if (name == Cmd::kRecordStart)      return RecordStartCmd{};
        if (name == Cmd::kRecordStop)       return RecordStopCmd{};
        if (name == Cmd::kPlaybackStart)    return PlaybackStartCmd{};
        if (name == Cmd::kPlaybackStop)     return PlaybackStopCmd{};
        if (name == Cmd::kPlaybackPause)    return PlaybackPauseCmd{};
        if (name == Cmd::kPlaybackResume)   return PlaybackResumeCmd{};
        if (name == Cmd::kPlaybackSetSpeed) return PlaybackSetSpeedCmd{args};
        if (name == Cmd::kPlaybackSeek)     return PlaybackSeekCmd{args};
        if (name == Cmd::kPlaybackStep)     return PlaybackStepCmd{args};
        if (name == Cmd::kPlaybackSetLoop)  return PlaybackSetLoopCmd{args};
        if (name == Cmd::kPlaybackGetInfo)  return PlaybackGetInfoCmd{};

        return StopCmd{};
    }
}
