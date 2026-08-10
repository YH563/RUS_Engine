#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "rus_sim_bridge/bridge_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = rus_sim_bridge::BridgeNode::Create();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
