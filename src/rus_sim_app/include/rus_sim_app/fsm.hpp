#pragma once

#include <memory>
#include <string>

#include "yasmin/state_machine.hpp"

#include "coordinator.hpp"

namespace RusSimApp {

    /**
     * @brief 应用状态机（基于 YASMIN）
     *
     * 顶层结构：
     *   INIT → IDLE →（PRE_SCAN | SCANNING）→ IDLE
     *   错误统一走 ERROR 复位。
     *
     * 状态通过协调器执行业务动作（触发 planning、等待结果），
     * 不直接接触 ROS。
     */
    class AppFsm {
    public:
        /**
         * @param coord 协调器（app 内唯一 ROS2 节点提供）
         */
        explicit AppFsm(std::shared_ptr<Coordinator> coord);

        /**
         * @brief 阻塞运行状态机
         *
         * @return 状态机最终 outcome
         */
        std::string Run();

        /** @brief 取消当前状态机执行 */
        void Cancel();

        /**
         * @brief 访问根状态机
         *
         * @return 根状态机共享指针
         */
        std::shared_ptr<yasmin::StateMachine> Root() const { return root_; }

    private:
        std::shared_ptr<Coordinator> coord_;
        std::shared_ptr<yasmin::StateMachine> root_;
    };

}  // namespace RusSimApp
