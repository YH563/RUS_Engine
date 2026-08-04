#include "rclcpp/rclcpp.hpp"

#include "rus_sim_app/coordinator.hpp"

// app 端唯一的 ROS2 节点 = 协调器（Coordinator）
// 负责：持有 YASMIN 状态机（AppFsm）+ user_interface（app WebSocket）
// 具体协调器实现位于 coordinator.cpp。
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    // 协调器单例节点（app 唯一节点，内含 yasmin executor）
    auto coord = RusSimApp::Coordinator::get_instance();

    // TODO: 组装 AppFsm(coord) 并启动状态机线程
    // TODO: 启动 user_interface（app WebSocket 服务器）

    RCLCPP_INFO(coord->get_logger(), "rus_sim_app 协调器节点已启动");

    rclcpp::spin(coord);
    rclcpp::shutdown();
    return 0;
}
