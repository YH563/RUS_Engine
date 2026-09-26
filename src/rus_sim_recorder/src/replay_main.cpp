#include "rus_sim_replayer/replay_node.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RusRecorder::ReplayNode>();
    RCLCPP_INFO(node->get_logger(), "ReplayNode 已启动（指令服务 /replayer/command）");

    // 单线程执行器：服务回调只改状态（replay_step 会在回调里逐条发布，见节点注释），
    // 时间轴节拍与真正的发布都在节点内部的回放线程 —— 全进程只有它一个发布者。
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;  // node 析构 → 停回放线程 → 关文件（录音文件只读，不会被改动）
}
