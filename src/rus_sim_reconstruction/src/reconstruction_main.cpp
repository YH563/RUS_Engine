#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "rus_sim_reconstruction/reconstruction_node.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RusReconstruction::ReconstructionNode>());
    rclcpp::shutdown();
    return 0;
}
