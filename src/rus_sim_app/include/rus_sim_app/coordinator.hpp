#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <unordered_map>
#include <variant>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rus_sim_interfaces/srv/command_service.hpp"
#include "rus_sim_utils/types.hpp"
#include "rus_sim_utils/ws_server.hpp"

#include "command_mapper.hpp"
#include "command_router.hpp"

namespace RusSimApp {

    // ── 统一请求 ──

    /** 统一请求 = 高层指令（操作 + 状态查询统一承载，见 rus_sim_utils/types.hpp） */
    using AppRequest = RusUtils::HighLevelCommand;

    // ── 任务结果 ──

    /** 任务状态 */
    enum class TaskStatus {
        SUCCESS,      // 成功
        FAILED,       // 失败
        TIMEOUT,      // 超时
        INTERRUPTED,  // 被打断
        CANCELLED,    // 取消
    };

    /** 任务结果（统一承载执行结果与查询数据） */
    struct TaskResult {
        TaskStatus status = TaskStatus::FAILED;
        std::string message;
        std::vector<double> data;  // 查询数据（如 is_motion_done 0/1）
    };

    /** 任务句柄 */
    using TaskHandle = uint64_t;

    // ── 协调器 ──

    /**
     * @brief 协调器：app 端唯一的 ROS2 节点
     *
     * 职责：
     *   1. 抽象任务接口：SubmitTask / WaitTask / CancelTask（操作与查询统一）
     *   2. 指令映射与转发：CommandMapper（高层 → 下游）+ CommandRouter（路由 + 异步等待）
     *   3. 前端入口：WsServer（业务指令 + 统一展示推送）
     *   4. 意图输入：WaitForUserCommand（fsm 等待用户指令）
     *
     * 运动指令由 planning 直接下发 driver，协调器不中转数据。
     */
    class Coordinator : public rclcpp::Node {
    public:
        using SharedPtr = std::shared_ptr<Coordinator>;

        /** 用户指令入口（AppFsm 注入：指令名 + 参数 → 业务事件） */
        using UserCommandSink = std::function<bool(const std::string&, const std::vector<double>&)>;

        /**
         * @brief 获取协调器单例（app 唯一节点）
         *
         * @return 协调器共享指针
         */
        static SharedPtr get_instance();

        // ── 抽象任务接口（fsm 使用，操作与查询统一） ──

        /**
         * @brief 提交任务（操作指令或状态查询），异步执行
         *
         * @param req 统一请求
         * @return 任务句柄（用于 WaitTask / CancelTask）
         */
        TaskHandle SubmitTask(const AppRequest& req);

        /**
         * @brief 阻塞等待任务结果
         *
         * @param handle  任务句柄
         * @param timeout 超时 [s]
         * @return 任务结果（状态 + 数据）
         */
        TaskResult WaitTask(const TaskHandle& handle, double timeout);

        /**
         * @brief 取消任务
         *
         * @param handle 任务句柄
         */
        void CancelTask(const TaskHandle& handle);

        // ── 意图输入（fsm 等待用户指令） ──

        /**
         * @brief 等待用户业务指令
         *
         * @param timeout 超时 [s]
         * @return 指令字符串（见 RusUtils::Cmd 常量），超时返回 "timeout"
         */
        std::string WaitForUserCommand(double timeout);

        // ── 用户指令入口（前端 WS → 解析 → 任务） ──

        /**
         * @brief 处理用户侧指令
         *
         * 解析指令名 / 参数为 HighLevelCommand，提交为任务。
         *
         * @param name 指令名（见 RusUtils::Cmd 常量）
         * @param args 参数数组
         * @return true 处理成功，false 未知指令
         */
        bool HandleUserCommand(const std::string& name, const std::vector<double>& args);

        /**
         * @brief 注入用户指令入口（AppFsm 启动时调用）
         *
         * 前端 WS 指令经此转发到业务状态机（指令名 → 业务事件）。
         *
         * @param sink 指令回调
         */
        void SetUserCommandSink(UserCommandSink sink);

    protected:
        /** @brief 构造协调器（创建映射器 / 路由器 / 通道 / WsServer） */
        Coordinator();

    private:
        // 指令映射器（高层指令 → 下游子指令）
        std::unique_ptr<CommandMapper> mapper_;

        // 指令路由器（统一经 CommandService 服务转发 + 异步等待结果）
        std::unique_ptr<CommandRouter> router_;

        // 前端 WebSocket 服务器
        std::unique_ptr<RusUtils::WsServer> ws_;

        // 用户指令入口（AppFsm 注入，状态机事件入口）
        UserCommandSink user_sink_;

        // 任务表：句柄 → 结果（供 WaitTask 查询）
        mutable std::mutex tasks_mutex_;
        std::unordered_map<TaskHandle, std::shared_ptr<std::promise<TaskResult>>> tasks_;
        std::atomic<TaskHandle> next_handle_{0};

        // 业务执行默认超时 [s]
        static constexpr double kDefaultTimeout = 300.0;
    };

}  // namespace RusSimApp
