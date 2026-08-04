#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rus_sim_interfaces/srv/driver_command.hpp"
#include "rus_sim_utils/types.hpp"

namespace RusSimApp {

    // ── 协调器 ──

    /**
     * @brief 协调器：app 端唯一的 ROS2 节点（纯 node）
     *
     * 职责：
     *   1. 向下层（planning / driver）经话题发布控制指令（预扫查 / 扫查 / 停止）
     *   2. 状态主动获取：经 /driver/command 服务查询（is_motion_done）
     *   3. 业务状态维护：如预扫查是否完成，供状态机查询
     *   4. 前端交互：app WebSocket（user_interface）业务指令入口
     *
     * planning 与 driver 之间直接传递运动数据，协调器不中转。
     */
    class Coordinator : public rclcpp::Node {
    public:
        using SharedPtr = std::shared_ptr<Coordinator>;

        /**
         * @brief 获取协调器单例（app 唯一节点）
         *
         * @return 协调器共享指针
         */
        static SharedPtr get_instance();

        // ── 业务控制（内部经话题下发给 planning / driver） ──

        /**
         * @brief 执行预扫查
         *
         * 发布预扫查启动指令，阻塞等待 planning 完成。
         *
         * @param timeout 超时 [s]
         * @return true 预扫查成功，false 失败 / 超时
         */
        bool ExecutePreScan(double timeout);

        /**
         * @brief 执行正式扫查
         *
         * 发布扫查启动指令，阻塞等待 planning 完成。
         *
         * @param timeout 超时 [s]
         * @return true 扫查成功，false 失败 / 被打断
         */
        bool ExecuteScan(double timeout);

        /** @brief 停止当前扫查（发布停止指令，打断 planning） */
        void StopScan();

        // ── 状态查询 ──

        /**
         * @brief 查询预扫查是否已完成
         *
         * @return true 已完成（允许正式扫查）
         */
        bool IsPreScanDone() const;

        /**
         * @brief 查询当前动作是否完成
         *
         * 经 /driver/command 服务查询 is_motion_done，非订阅流。
         *
         * @return true 运动已完成，false 仍在运动中
         */
        bool IsMotionDone();

        // ── 用户指令入口（解析 + 自动分发） ──

        /**
         * @brief 处理用户侧指令
         *
         * 解析指令名 / 参数为 HighLevelCommand，经 std::visit 自动分发执行。
         *
         * @param name 指令名（见 RusUtils::Cmd 常量）
         * @param args 参数数组
         * @return true 处理成功，false 未知指令
         */
        bool HandleUserCommand(const std::string& name, const std::vector<double>& args);

        // ── 前端交互（阻塞等待） ──

        /**
         * @brief 等待用户业务指令
         *
         * @param timeout 超时 [s]
         * @return 指令字符串（如 "start_prescan"），超时返回 "timeout"
         */
        std::string WaitForUserCommand(double timeout);

    protected:
        /** @brief 构造协调器（创建控制话题发布器、driver 服务客户端） */
        Coordinator();

    private:
        /** @brief 发布控制指令（携带参数序列化）到 /app/control */
        bool publish_control(std::string_view name, const std::vector<double>& args);

        // 控制指令话题发布器（下发 planning / driver）
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr control_pub_;

        // driver 服务客户端（/driver/command）
        rclcpp::Client<rus_sim_interfaces::srv::DriverCommand>::SharedPtr driver_cmd_client_;

        // 业务状态：预扫查是否完成
        std::atomic<bool> prescan_done_{false};

        // 业务执行默认超时 [s]
        static constexpr double kDefaultTimeout = 300.0;
    };

}  // namespace RusSimApp
