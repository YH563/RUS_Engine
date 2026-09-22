#include "rus_sim_recorder/recorder_node.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RusRecorder::RecorderNode>();
    RCLCPP_INFO(node->get_logger(), "RecorderNode 已启动");

    // 单线程执行器足够：重活（CRC / 写盘）在节点内部的独立写线程，
    // 回调只做序列化 + 入队（微秒级），顺序执行即可，避免额外的并发面。
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;  // node 析构 → 写线程 join → 尾索引落盘
}
