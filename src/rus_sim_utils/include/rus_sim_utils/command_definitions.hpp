#pragma once

#include <string>
#include <string_view>

namespace RusUtils
{
    /**
     * @brief 面向用户侧执行的高层指令
     * 
     */
    namespace Cmd
    {
        // 开始进行预扫查
        inline constexpr std::string_view kPreScanStart = "pre_scan_start";
        // 预扫查完成
        inline constexpr std::string_view kPreScanEnd = "pre_scan_end";
        // 设置起始位姿
        inline constexpr std::string_view kSetStartPose = "set_start_pose";
        // 设置终点位姿
        inline constexpr std::string_view kSetEndPose = "set_end_pose";
        // 进行轨迹规划
        inline constexpr std::string_view kPlan = "plan";
        // 执行规划好的轨迹
        inline constexpr std::string_view kExecute = "execute";
        // 停止所有运动（急停）
        inline constexpr std::string_view kStop = "stop";
        // 暂停扫查
        inline constexpr std::string_view kPause = "pause";
        // 继续扫查
        inline constexpr std::string_view kResume = "resume";
        // 错误复位
        inline constexpr std::string_view kReset = "reset";
        // 连接驱动、上使能
        inline constexpr std::string_view kConnect = "connect";
        // 关闭系统
        inline constexpr std::string_view kShutdown = "shutdown";
        // 查询：预扫查是否完成
        inline constexpr std::string_view kQueryPreScanDone = "query_prescan_done";
        // 查询：运动是否完成
        inline constexpr std::string_view kQueryMotionDone = "query_motion_done";

        // 开始录制运动数据
        inline constexpr std::string_view kRecordStart = "record_start";
        // 停止录制
        inline constexpr std::string_view kRecordStop = "record_stop";
        // 开始回放录制数据
        inline constexpr std::string_view kPlaybackStart = "playback_start";
        // 停止回放
        inline constexpr std::string_view kPlaybackStop = "playback_stop";
        // 暂停回放
        inline constexpr std::string_view kPlaybackPause = "playback_pause";
        // 继续回放
        inline constexpr std::string_view kPlaybackResume = "playback_resume";
        // 设置回放速度倍率，args = [speed]
        inline constexpr std::string_view kPlaybackSetSpeed = "playback_set_speed";
        // 跳转回放时间，args = [time_seconds]
        inline constexpr std::string_view kPlaybackSeek = "playback_seek";
        // 逐帧步进，args = [direction]
        inline constexpr std::string_view kPlaybackStep = "playback_step";
        // 设置循环播放，args = [enable]
        inline constexpr std::string_view kPlaybackSetLoop = "playback_set_loop";
        // 查询回放信息
        inline constexpr std::string_view kPlaybackGetInfo = "playback_get_info";
    }
}