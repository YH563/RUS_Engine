#include "rus_sim_driver/driver_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RusDriverNode::DriverNode>();
    RCLCPP_INFO(node->get_logger(), "DriverNode 已启动");

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}