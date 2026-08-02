#pragma once

#include <string>
#include <string_view>

namespace RusUtils
{
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
    }
}