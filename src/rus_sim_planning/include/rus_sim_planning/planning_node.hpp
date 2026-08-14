#pragma once

// ════════════════════════════════════════════════════════════════════
//  规划节点
//  ────────────────────────────────────────────────────────────────────
//  持有两个核心模块，负责数据传递、整合与转发：
//    - 轨迹生成模块   TrajectoryGenerator      （点云路径规划）
//    - 插值计算模块   TrajectoryInterpolator   （路径稠密化）
//
//  对外指令（/planning/command 服务，供 bridge/客户端调用）：
//    set_start_pose [x,y,z] | [x,y,z,rx,ry,rz] | [x,y,z,qx,qy,qz,qw]
//    set_end_pose   同上
//    plan           —— 生成稀疏路径 + 插值稠密化（只规划，不执行）
//    execute        —— 开始伺服执行：按 servo_rate_hz 逐点下发 servo_cart
//    stop           —— 停止伺服执行
//
//  数据流：
//    点云话题 ──────→ 轨迹生成模块（建图/法线）
//    /driver/state ──→ 插值计算模块（机械臂状态）
//    set_start/end_pose → 起终点
//    plan ────────→ generator 生成 → interpolator 插值
//    execute ─────→ 伺服定时器按 servo_rate_hz 逐点转发 servo_cart → /driver/command
// ════════════════════════════════════════════════════════════════════

#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include "rus_sim_interfaces/msg/module_event.hpp"
#include "rus_sim_interfaces/msg/robot_state.hpp"
#include "rus_sim_interfaces/srv/command_service.hpp"

#include "rus_sim_planning/trajectory_generator.hpp"
#include "rus_sim_planning/trajectory_interpolator.hpp"

namespace RusSimPlanning {

    /**
     * @brief 规划节点
     *
     * 持有轨迹生成 + 插值计算两个核心模块，通过 /planning/command
     * 接收外部指令（起终点/规划/执行），以伺服指令（servo_cart）
     * 按 servo_rate_hz 频率转发给驱动跟踪执行。
     */
    class PlanningNode : public rclcpp::Node {
    public:
        PlanningNode();

        // ── 对外触发接口（指令回调 / 控制算法调用）──

        /**
         * @brief 规划：生成稀疏路径 + 插值稠密化（不执行）
         *
         * @param client_id 触发指令的前端 command id（用于 plan_done 事件关联）
         * @return true 成功；false 未完成预扫查 / 起终点未设置 / 生成失败
         */
        bool Plan(uint32_t client_id = 0);

        /**
         * @brief 开始伺服执行：按 servo_rate_hz 逐点下发 servo_cart 给驱动跟踪
         *
         * @param client_id 触发指令的前端 command id（用于 scan_done 事件关联）
         * @return true 成功启动
         */
        bool Execute(uint32_t client_id = 0);

        /**
         * @brief 停止伺服执行（下发 servo_end + driver stop）
         */
        void StopExecution();

        /**
         * @brief 暂停伺服执行：停掉伺服下发并标记暂停。
         * driver 之后可能上报运动完成，但 planning 知道是暂停，
         * query_motion_done 仍返回未完成。
         *
         * @return true 已暂停；false 未在执行中（无可暂停）
         */
        bool Pause();

        /**
         * @brief 恢复被暂停的伺服执行（从暂停位置继续）
         *
         * @return true 已恢复；false 未在暂停中或轨迹已执行完
         */
        bool Resume();

        /**
         * @brief 复位规划状态：停止伺服、清空轨迹、状态归零
         */
        void ResetState();

        /**
         * @brief 设置起点 / 终点（外部指令 set_start_pose / set_end_pose）
         *
         * @param pose 起点 / 终点位姿（法兰系，单位 m/rad）
         */
        void SetStartPose(const geometry_msgs::msg::Pose& pose) { start_pose_ = pose; }
        void SetEndPose(const geometry_msgs::msg::Pose& pose)   { goal_pose_  = pose; }

        /**
         * @brief 模块访问（调试 / 控制算法集成用）
         */
        TrajectoryGenerator& Generator() { return generator_; }
        TrajectoryInterpolator& Interpolator() { return interpolator_; }

    private:
        // ── 指令服务回调（/planning/command）──
        void handle_command(
            const std::shared_ptr<rmw_request_id_t> req_header,
            const std::shared_ptr<rus_sim_interfaces::srv::CommandService::Request> req,
            std::shared_ptr<rus_sim_interfaces::srv::CommandService::Response> res);

        // ── 数据获取 ──
        void on_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
        void on_driver_state(const rus_sim_interfaces::msg::RobotState::SharedPtr msg);

        // ── 伺服执行 ──
        void servo_tick();                                  // 定时器：逐点下发
        void stop_servo();                                  // 停止伺服（不发事件）
        void start_servo_timer();                           // 启动 / 复位伺服定时器
        void send_servo_cart(const geometry_msgs::msg::Pose& pose);
        void send_cmd_async(const std::string& cmd, const std::vector<double>& args);

        // ── 事件发布（→ /module_events，bridge 订阅后广播为前端 event）──
        void publish_event(std::string_view event, bool success,
                           std::string_view message,
                           const std::vector<double>& result,
                           uint32_t client_id);

        // ── 参数 ──
        std::string point_cloud_topic_;
        std::string driver_state_topic_;
        std::string driver_command_service_;
        double servo_rate_hz_ = 125.0;    // 伺服指令发布频率 [Hz]
        int interpolate_points_ = 10;     // 每段插值点数

        std::optional<geometry_msgs::msg::Pose> start_pose_;
        std::optional<geometry_msgs::msg::Pose> goal_pose_;
        bool prescan_done_ = false;             // 预扫查是否完成（点云已就绪）
        uint32_t execute_client_id_ = 0;        // execute 指令的 client_id（scan_done 事件关联）

        // ── 核心模块 ──
        TrajectoryGenerator generator_;        // 轨迹生成模块
        TrajectoryInterpolator interpolator_;  // 插值计算模块

        bool executing_ = false;               // 伺服执行中
        bool paused_ = false;                  // 伺服执行已暂停（driver 可能报完成，planning 报未完成）

        // ── ROS 通信 ──
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
        rclcpp::Subscription<rus_sim_interfaces::msg::RobotState>::SharedPtr state_sub_;
        rclcpp::Client<rus_sim_interfaces::srv::CommandService>::SharedPtr driver_cmd_client_;
        rclcpp::Service<rus_sim_interfaces::srv::CommandService>::SharedPtr cmd_server_;
        rclcpp::TimerBase::SharedPtr servo_timer_;
        rclcpp::Publisher<rus_sim_interfaces::msg::ModuleEvent>::SharedPtr event_pub_;
    };

}  // namespace RusSimPlanning
