#include <rclcpp/rclcpp.hpp>

#include "rus_sim_data/data_node.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RusSimData::DataNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
