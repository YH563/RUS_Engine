#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rus_sim_utils/types.hpp"

#include "coordinator.hpp"
#include "fsm_core.hpp"
#include "states_def.hpp"

namespace RusSimApp {

    /**
     * @brief 应用状态机（业务调度层）
     *
     * 组装事件驱动 Fsm<State, Event>：
     *   - 转移表 + 生命周期回调（on_enter / on_exit）+ 抢占回调（on_preempt）
     *   - 状态动作经协调器执行（SubmitTask / WaitTask / CancelTask）
     *   - 前端指令 → 业务事件映射（HandleUserCommand）
     *
     * 依赖方向：AppFsm → Coordinator（单向，无循环）。
     * 查询指令（query_*）不过状态机，直接经协调器转发。
     */
    class AppFsm {
    public:
        using SharedPtr = std::shared_ptr<AppFsm>;

        /**
         * @param coord 协调器（app 内唯一 ROS2 节点）
         */
        explicit AppFsm(Coordinator::SharedPtr coord);

        // ── 生命周期 ──

        /** @brief 注入指令入口、设初态、启动状态机线程 */
        void Start();

        /** @brief 停止状态机线程 */
        void Stop();

        // ── 事件入口（前端指令 → 事件） ──

        /**
         * @brief 用户指令入口（协调器 WS 注入）
         *
         * 指令名 → 业务事件（Post / Preempt）；查询指令直接经协调器转发。
         *
         * @param name 指令名（见 RusUtils::Cmd 常量）
         * @param args 参数数组
         * @return true 已识别，false 未知指令
         */
        bool HandleUserCommand(const std::string& name, const std::vector<double>& args);

        /** @brief 投递普通业务事件（入队） */
        void HandleEvent(Event evt);

        /** @brief 投递抢占业务事件（如 STOP，立即执行抢占回调 + 优先转移） */
        void HandlePreempt(Event evt);

        // ── 查询 ──

        /** @brief 当前业务状态 */
        State Current() const;

        /** @brief 状态机线程是否在运行 */
        bool IsRunning() const;

    private:
        /** @brief 配置转移表 + 生命周期/抢占回调 */
        void configure();

        /**
         * @brief 启动主任务：提交任务 + 监视线程（结果 → 业务事件）
         *
         * 仅用于"进入执行态"提交的主任务（预扫查 / 正式扫查）。
         */
        void start_main_task(const RusUtils::HighLevelCommand& cmd, double timeout);

        /** @brief 下发控制指令（Pause/Stop/Shutdown，fire-and-forget，结果忽略） */
        void send_control(const RusUtils::HighLevelCommand& cmd);

        /** @brief 取消当前主任务（on_exit 时） */
        void cancel_main_task();

        Coordinator::SharedPtr coord_;
        std::unique_ptr<Fsm<State, Event>> fsm_;
        std::thread fsm_thread_;

        // 当前主任务句柄（0 = 无）
        std::mutex task_mutex_;
        TaskHandle active_handle_{0};

        std::atomic<bool> running_{false};
    };

}  // namespace RusSimApp
