#include "rclcpp/rclcpp.hpp"

#include "rus_sim_app/app_fsm.hpp"
#include "rus_sim_app/coordinator.hpp"

// app 端唯一的 ROS2 节点 = 协调器（Coordinator）
// 业务状态机（AppFsm）持有协调器引用，在独立线程中运行。
// 前端 WebSocket 服务器由协调器承载（TODO: 接入 user_interface）。
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    // 协调器单例节点（app 唯一节点）
    auto coord = RusSimApp::Coordinator::get_instance();

    // 业务状态机：转移表 + 生命周期/抢占回调 + 指令→事件映射
    auto fsm = std::make_shared<RusSimApp::AppFsm>(coord);
    fsm->Start();

    RCLCPP_INFO(coord->get_logger(), "rus_sim_app 协调器节点已启动");

    rclcpp::spin(coord);

    fsm->Stop();
    rclcpp::shutdown();
    return 0;
}
