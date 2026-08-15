#include "rus_sim_perception/perception_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RusPerception::PerceptionNode>();
    RCLCPP_INFO(node->get_logger(), "PerceptionNode 已启动");

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}

