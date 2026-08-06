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

    // 统一指令承载
    using HighLevelCommand = std::variant<
        QueryMotionDone, QueryPreScanDone, PreScanStartCmd, SetStartPoseCmd, SetEndPoseCmd, PlanCmd, ExecuteCmd,
        StopCmd, PauseCmd, ResumeCmd, ResetCmd, ConnectCmd, ShutdownCmd>;


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

        return StopCmd{};
    }
}
