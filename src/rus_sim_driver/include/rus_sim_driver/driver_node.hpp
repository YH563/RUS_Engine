#pragma once

#include <memory>
#include <string>
#include <vector>
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

        /**
         * @brief 取点动单次位移上限（来自 driver_params.yaml 配置）
         *
         * 单位与指令层一致：rad（关节点动 JOG_0 / 笛卡尔旋转轴 nb=4~6）、
         * m（笛卡尔平移轴 nb=1~3）；0 = 不限制。
         * 前端 start_jog 传入的 max_dis 仅解析、不参与控制，实际上限一律取本配置。
         *
         * @param type MotionCommand.type（MOTION_TYPE_JOG_0/1/2）
         * @param axis 点动轴号 nb（1~6）
         */
        double jog_limit_for(uint8_t type, uint8_t axis) const;

        // 执行指令文件
        bool run_script(const std::string& path);

        // === 工具坐标系配置持久化 ===

        /**
         * @brief 从 tool_coords_file 加载工具坐标系配置（不存在时用内置默认并创建文件）
         *
         * 配置格式（简单 yaml）：

         *   tool_coords:

         *     0: [x,y,z,rx,ry,rz]

         *     ...

         *   tool_index: N

         */
        void load_tool_coords();

        /** @brief 将内存工具坐标系表与当前索引写回 tool_coords_file（持久化，防重启丢失） */
        bool save_tool_coords();

        /**
         * @brief 初始化工具坐标系：批量 SetToolCoord 写入驱动 + SetToolIndex 生效
         *
         * 真实驱动：将配置写入控制器（机械臂重启后可恢复）；
         * 仿真驱动：填充本地工具变换矩阵表并同步运动学。
         */
        void init_tool_coords();

        // 驱动实例
        std::unique_ptr<RusRobotDriver::IRobotDriver> driver_;
        bool is_sim_{false};
        std::string robot_ip_;  // 机器人 IP（Connect 指令使用）

        // 文件路径参数（由构造函数 declare_parameter，供 dispatch 使用）
        std::string script_path_;

        // 工具坐标系配置（索引 → [x,y,z,rx,ry,rz]，m/rad）
        std::vector<std::vector<double>> tool_coords_;
        int tool_index_param_{0};     // 当前工具坐标系索引（持久化）
        std::string tool_coords_file_;  // 工具坐标系配置文件路径

        // 点动（start_jog）单次位移上限（driver_params.yaml；单位 rad = 关节/旋转轴，m = 平移轴；0 = 不限制）
        double jog_max_dis_joint_{1.5708};  // 关节点动 JOG_0（90°）
        double jog_max_dis_trans_{0.15};    // 笛卡尔平移（JOG_1/2 轴 1~3，150 mm）
        double jog_max_dis_rot_{1.5708};    // 笛卡尔旋转（JOG_1/2 轴 4~6，90°）

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
