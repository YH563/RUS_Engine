#pragma once

#include <cstdint>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

#include "rus_sim_data/command_defs.hpp"

namespace RusSimData {

    // ── 录制指令 ──
    struct RecordStartCmd {};
    struct RecordStopCmd {};

    // ── 回放指令 ──
    struct PlaybackStartCmd    {};
    struct PlaybackStopCmd     {};
    struct PlaybackPauseCmd    {};
    struct PlaybackResumeCmd   {};
    struct PlaybackSetSpeedCmd { double speed = 1.0; };
    struct PlaybackSeekCmd     { double time_seconds = 0.0; };
    struct PlaybackStepCmd     { int8_t direction = 1; };
    struct PlaybackSetLoopCmd  { bool enable = false; };
    struct PlaybackGetInfoCmd  {};

    // 统一指令承载
    using DataCommand = std::variant<
        RecordStartCmd, RecordStopCmd,
        PlaybackStartCmd, PlaybackStopCmd, PlaybackPauseCmd, PlaybackResumeCmd,
        PlaybackSetSpeedCmd, PlaybackSeekCmd, PlaybackStepCmd, PlaybackSetLoopCmd,
        PlaybackGetInfoCmd>;

    // std::visit 辅助模板
    template<class... Ts> struct Overloaded : Ts... { using Ts::operator()...; };
    template<class... Ts> Overloaded(Ts...) -> Overloaded<Ts...>;

    /**
     * @brief 将指令名和参数数组解析为 DataCommand
     *
     * @param name 指令名（见 command_defs.hpp 的 Cmd 常量）
     * @param args float64[] 参数数组
     * @return DataCommand
     *
     * 参数格式说明：
     *   playback_set_speed [speed]
     *   playback_seek      [time_seconds]
     *   playback_step      [direction]
     *   playback_set_loop  [enable]
     */
    inline DataCommand ParseDataCommand(std::string_view name, const std::vector<double>& args)
    {
        using namespace RusSimData::Cmd;

        if (name == kRecordStart)        return RecordStartCmd{};
        if (name == kRecordStop)         return RecordStopCmd{};
        if (name == kPlaybackStart)      return PlaybackStartCmd{};
        if (name == kPlaybackStop)       return PlaybackStopCmd{};
        if (name == kPlaybackPause)      return PlaybackPauseCmd{};
        if (name == kPlaybackResume)     return PlaybackResumeCmd{};
        if (name == kPlaybackSetSpeed)   return PlaybackSetSpeedCmd{args.empty() ? 1.0 : args[0]};
        if (name == kPlaybackSeek)       return PlaybackSeekCmd{args.empty() ? 0.0 : args[0]};
        if (name == kPlaybackStep)       return PlaybackStepCmd{args.empty() ? int8_t{1} : static_cast<int8_t>(args[0])};
        if (name == kPlaybackSetLoop)    return PlaybackSetLoopCmd{args.empty() || args[0] != 0.0};
        if (name == kPlaybackGetInfo)    return PlaybackGetInfoCmd{};

        // 未知指令：返回 RecordStopCmd 兜底（与驱动 ParseCommand 一致）
        return RecordStopCmd{};
    }

}  // namespace RusSimData
