#pragma once

#include <optional>
#include <string_view>

#include "rus_sim_utils/command_definitions.hpp"

namespace RusSimApp {

    /** 业务状态 */
    enum class State {
        INIT,               // 初始化
        IDLE,               // 待机
        PRE_SCAN_RUNNING,   // 预扫查-执行中（点云建模 + 起终点 + 路径生成）
        PRE_SCAN_PAUSED,    // 预扫查-暂停
        SCANNING_RUNNING,   // 正式扫查-执行中
        SCANNING_PAUSED,    // 正式扫查-暂停
        ERROR,              // 错误
    };

    /** 业务事件（外部指令 + 内部任务结果） */
    enum class Event {
        // ── 外部用户指令 ──
        START_PRE_SCAN,     // 开始预扫查
        START_SCAN,         // 开始扫查
        STOP,               // 停止（急停）
        PAUSE,              // 暂停
        RESUME,             // 继续
        RESET,              // 错误复位
        SHUTDOWN,           // 关闭

        // ── 内部任务结果 ──
        TASK_DONE,          // 任务成功
        TASK_FAILED,        // 任务失败
        TASK_TIMEOUT,       // 任务超时
        TASK_CANCELLED,     // 任务取消
    };

    /**
     * @brief 高层指令名 → 业务事件
     *
     * 仅映射"有对应业务事件"的指令；无对应事件的指令
     * （如查询指令 query_*、参数类指令 set_* / plan / connect 等）
     * 返回 std::nullopt，调用方应绕过状态机另行处理。
     *
     * @param name 指令名（见 RusUtils::Cmd 常量）
     * @return 对应业务事件，无映射返回 std::nullopt
     */
    inline std::optional<Event> StringToEvent(std::string_view name)
    {
        namespace Cmd = RusUtils::Cmd;

        if (name == Cmd::kPreScanStart) return Event::START_PRE_SCAN;
        if (name == Cmd::kExecute)      return Event::START_SCAN;
        if (name == Cmd::kPause)        return Event::PAUSE;
        if (name == Cmd::kResume)       return Event::RESUME;
        if (name == Cmd::kReset)        return Event::RESET;
        if (name == Cmd::kStop)         return Event::STOP;
        if (name == Cmd::kShutdown)     return Event::SHUTDOWN;

        return std::nullopt;
    }

    /**
     * @brief 业务事件 → 高层指令名
     *
     * 仅映射外部用户指令类事件；内部任务结果事件（TASK_*）无对应
     * 指令名，返回空字符串。
     *
     * @param evt 业务事件
     * @return 指令名（见 RusUtils::Cmd 常量），无映射返回空字符串
     */
    inline std::string_view EventToString(Event evt)
    {
        namespace Cmd = RusUtils::Cmd;

        switch (evt) {
            case Event::START_PRE_SCAN: return Cmd::kPreScanStart;
            case Event::START_SCAN:     return Cmd::kExecute;
            case Event::PAUSE:          return Cmd::kPause;
            case Event::RESUME:         return Cmd::kResume;
            case Event::RESET:          return Cmd::kReset;
            case Event::STOP:           return Cmd::kStop;
            case Event::SHUTDOWN:       return Cmd::kShutdown;
            default:                    return "";
        }
    }

}  // namespace RusSimApp
