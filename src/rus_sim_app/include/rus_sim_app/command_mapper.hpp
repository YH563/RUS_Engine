#pragma once

#include <string>
#include <vector>

#include "rus_sim_utils/types.hpp"

namespace RusSimApp {

    /** 下游目标模块 */
    enum class Module {
        PLANNING,    // 运动规划 / 执行
        DRIVER,      // 驱动
        PERCEPTION,  // 感知（点云建模）
        DATA,        // 数据（录制 / 回放）
    };

    /** 一条下游子指令 */
    struct DownstreamCmd {
        Module target;             // 发给谁
        std::string name;          // 子模块指令名
        std::vector<double> args;  // 参数
    };

    /**
     * @brief 指令映射器：高层指令 → 下游子指令列表
     *
     * 只负责"指令解析 / 映射"，不负责发送。
     * 数据由下游模块之间直接传递，本层不转发数据。
     */
    class CommandMapper {
    public:
        /**
         * @brief 将高层指令映射为下游子指令列表
         *
         * 一个高层指令可能拆成多条下游指令
         * （如 stop 需要同时发给 planning 与 driver）。
         *
         * @param cmd 高层指令
         * @return 下游子指令列表
         */
        std::vector<DownstreamCmd> Map(const RusUtils::HighLevelCommand& cmd) const;
    };

}  // namespace RusSimApp
