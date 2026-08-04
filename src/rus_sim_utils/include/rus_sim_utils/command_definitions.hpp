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
    }
}