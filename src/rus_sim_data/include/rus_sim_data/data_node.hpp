#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <atomic>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <rus_sim_interfaces/msg/robot_state.hpp>
#include <rus_sim_interfaces/srv/command_service.hpp>
#include <rus_sim_utils/robot_state.hpp>
#include <rus_sim_utils/ws_server.hpp>

#include "rus_sim_data/data_recorder.hpp"
#include "rus_sim_data/playback_controller.hpp"
#include "rus_sim_data/types.hpp"

namespace RusSimData {

    /**
     * @brief 数据节点（录制 / 回放统一管理）
     *
     * 职责：
     *   1. 录制：订阅 /driver/state，录制指令期间将帧写入文件
     *   2. 回放：加载录制文件，按时间推进，发布 /data/state + /joint_states
     *   3. 前端：WsServer 广播回放状态 JSON（与驱动解耦，不经过驱动）
     *
     * 指令入口：Service /data/command（record_* / playback_*）
     */
    class DataNode : public rclcpp::Node {
    public:
        DataNode();

    private:
        // 服务回调：处理 /data/command
        void handle_command(
            const std::shared_ptr<rmw_request_id_t> req_header,
            const std::shared_ptr<rus_sim_interfaces::srv::CommandService::Request> req,
            std::shared_ptr<rus_sim_interfaces::srv::CommandService::Response> res);

        // 状态订阅回调：录制期间采集帧
        void on_driver_state(const rus_sim_interfaces::msg::RobotState::SharedPtr msg);

        // 回放定时器：推进回放并发布状态
        void playback_tick();

        // 指令分发
        bool dispatch(const std::string& cmd,
                      const std::vector<double>& args,
                      std::vector<double>& result);

        // 工具
        static rus_sim_interfaces::msg::RobotState to_ros_state(const RusUtils::RobotState& s);
        std::string state_to_json(const RusUtils::RobotState& state);

        // 录制器 / 回放控制器
        DataRecorder recorder_;
        std::vector<RusUtils::RobotState> playback_frames_;
        PlaybackController playback_ctrl_;

        // 录制路径 / 回放路径（参数）
        std::string record_path_;
        std::string playback_path_;

        // 回放推进
        std::atomic<bool> playback_active_{false};
        std::atomic<bool> recording_{false};
        std::chrono::steady_clock::time_point last_tick_;

        // ROS2 接口
        rclcpp::Service<rus_sim_interfaces::srv::CommandService>::SharedPtr command_server_;
        rclcpp::Subscription<rus_sim_interfaces::msg::RobotState>::SharedPtr state_sub_;
        rclcpp::Publisher<rus_sim_interfaces::msg::RobotState>::SharedPtr playback_state_pub_;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
        rclcpp::TimerBase::SharedPtr playback_timer_;

        // WebSocket 广播（回放状态推送前端）
        RusUtils::WsServer ws_server_;

        // 回放帧数据锁
        std::mutex playback_mutex_;
    };

}  // namespace RusSimData
