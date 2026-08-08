#pragma once

#include <string_view>

namespace RusSimData {

    /**
     * @brief 数据层指令字符串常量
     *
     * 录制 / 回放指令在此统一定义，供 DataNode 服务回调解析使用。
     * 与 rus_sim_driver 的指令定义解耦，前端经 app 路由到 /data/command。
     */
    namespace Cmd {

        // ── 录制 ──
        inline constexpr std::string_view kRecordStart   = "record_start";
        inline constexpr std::string_view kRecordStop    = "record_stop";

        // ── 回放 ──
        inline constexpr std::string_view kPlaybackStart = "playback_start";
        inline constexpr std::string_view kPlaybackStop  = "playback_stop";
        inline constexpr std::string_view kPlaybackPause      = "playback_pause";
        inline constexpr std::string_view kPlaybackResume     = "playback_resume";
        inline constexpr std::string_view kPlaybackSetSpeed   = "playback_set_speed";
        inline constexpr std::string_view kPlaybackSeek       = "playback_seek";
        inline constexpr std::string_view kPlaybackStep       = "playback_step";
        inline constexpr std::string_view kPlaybackSetLoop    = "playback_set_loop";
        inline constexpr std::string_view kPlaybackGetInfo    = "playback_get_info";

    }  // namespace Cmd

}  // namespace RusSimData
