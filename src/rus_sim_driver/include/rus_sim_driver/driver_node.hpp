#pragma once

#include <memory>
#include <string>
#include <fstream>
#include <sstream>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <rus_sim_interfaces/srv/command_service.hpp>
#include <rus_sim_interfaces/msg/robot_state.hpp>

#include "components/command_defs.hpp"
#include "driver/robot_driver.hpp"
#include "driver/sim_driver.hpp"

namespace RusSimRobotDriver { class RobotSimDriver; }

namespace RusDriverNode {
    /**
     * @brief ROS2 驱动节点
     *
     * Service /driver/command — 统一指令接口（由桥接层 CommandDispatcher 调用）
     * Topic  /driver/state    — 125Hz 发布 RobotState（桥接层订阅并转发到 WS /state）
     * Topic  /joint_states    — 发布 JointState（供 RViz 仿真可视化）
     *
     * 不持有 WebSocket 服务器：前后端通信统一走 rus_sim_bridge（纯网关）。
     */
    class DriverNode : public rclcpp::Node {
    public:
        DriverNode();
        ~DriverNode();

    private:
        // 服务回调：处理 /driver/command
        void handle_command(
            const std::shared_ptr<rmw_request_id_t> req_header,
            const std::shared_ptr<rus_sim_interfaces::srv::CommandService::Request> req,
            std::shared_ptr<rus_sim_interfaces::srv::CommandService::Response> res
        );

        // 定时器回调：发布 /driver/state
        void publish_state();

        // 根据命令名称分发到对应驱动接口
        bool dispatch(const std::string& cmd,
                      const std::vector<double>& args,
                      std::vector<double>& result);

        // 运行时切换驱动（switch_driver 指令）
        bool switch_driver_impl(uint8_t type, const std::string& ip);

        // 执行指令文件
        bool run_script(const std::string& path);

        // 驱动实例
        std::unique_ptr<RusRobotDriver::IRobotDriver> driver_;
        bool is_sim_{false};
        std::string robot_ip_;  // 机器人 IP（Connect 指令使用）

        // 文件路径参数（由构造函数 declare_parameter，供 dispatch 使用）
        std::string script_path_;

        // 安全获取仿真驱动引用（仅在 is_sim_==true 时调用，实现在 .cpp）
        RusSimRobotDriver::RobotSimDriver& sim_driver();

        // ROS2 接口
        rclcpp::Service<rus_sim_interfaces::srv::CommandService>::SharedPtr command_server_;
        rclcpp::Publisher<rus_sim_interfaces::msg::RobotState>::SharedPtr state_pub_;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
        rclcpp::TimerBase::SharedPtr timer_;

        // 关节名称（用于 /joint_states）
        std::vector<std::string> joint_names_;
    };

}  // namespace RusDriverNode
