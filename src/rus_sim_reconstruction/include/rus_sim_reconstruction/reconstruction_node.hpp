#pragma once

// ════════════════════════════════════════════════════════════════════
//  重建节点（感知层嵌入）：订阅单帧点云（base_link）→ 增量面元融合 → 发布重建结果
//  ────────────────────────────────────────────────────────────────────
//  输入：默认 /perception/frame（单视角当前帧，已变换到 base_link）
//  输出：默认 /reconstructed_cloud（面元地图快照，PointCloud2 base_link）
//  已知位姿 → 输入已是 base_link，融合时 T=单位阵；法线可选估计。
// ════════════════════════════════════════════════════════════════════

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "rus_sim_reconstruction/sharded_surfel_map.hpp"

namespace RusReconstruction {

    class ReconstructionNode : public rclcpp::Node {
    public:
        ReconstructionNode();

    private:
        void OnCloud(sensor_msgs::msg::PointCloud2::SharedPtr msg);
        void PublishReconstruction();

        std::unique_ptr<ShardedSurfelMap> map_;

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
        rclcpp::TimerBase::SharedPtr timer_;

        std::string input_topic_;
        std::string output_topic_;
        double min_confidence_ = 2.0;
        double publish_period_ = 1.0;
        bool estimate_normals_ = true;
        int normal_k_ = 10;
        std::string save_pcd_;   // 非空则每次发布时落盘该 PCD（测试用）

        uint64_t frames_in_ = 0;
        uint64_t points_in_ = 0;
        uint64_t frames_dropped_ = 0;
    };

}  // namespace RusReconstruction
