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

#include "rus_sim_interfaces/msg/mesh_frame.hpp"
#include "rus_sim_interfaces/msg/sensor_frame.hpp"
#include "rus_sim_reconstruction/sharded_surfel_map.hpp"
#include "rus_sim_reconstruction/tsdf_volume.hpp"

namespace RusReconstruction {

    class ReconstructionNode : public rclcpp::Node {
    public:
        ReconstructionNode();

    private:
        void OnCloud(sensor_msgs::msg::PointCloud2::SharedPtr msg);
        void PublishReconstruction();
        void PublishMesh();
        void PublishPcMap();

        std::unique_ptr<ShardedSurfelMap> map_;
        std::unique_ptr<TsdfVolume> tsdf_;   // 增量网格（按块 upsert/remove；enable_mesh 时才积分/发布）

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;              // /reconstructed_cloud
        rclcpp::Publisher<rus_sim_interfaces::msg::SensorFrame>::SharedPtr pcmap_pub_; // 面元点云图（前端）
        rclcpp::Publisher<rus_sim_interfaces::msg::MeshFrame>::SharedPtr mesh_pub_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::TimerBase::SharedPtr mesh_timer_;
        rclcpp::TimerBase::SharedPtr pcmap_timer_;

        std::string input_topic_;
        std::string output_topic_;
        double min_confidence_ = 2.0;
        double publish_period_ = 1.0;
        bool estimate_normals_ = true;
        int normal_k_ = 10;
        std::string save_pcd_;   // 非空则每次发布时落盘该 PCD（测试用）

        // ── 面元点云图（前端 /pcmap 通道；点云图 = 融合后的面元，去噪/带置信度）──
        std::string pcmap_topic_;
        double pcmap_period_ = 1.0;          // 发布周期（秒）
        float  pcmap_min_confidence_ = 2.0f; // 只发置信度 ≥ 此值的面元
        bool   pcmap_compress_ = true;       // payload zstd 压缩
        uint32_t pcmap_seq_ = 0;

        // ── 增量网格（默认关；前端当前只收面元点云）──
        bool enable_mesh_ = false;
        std::string mesh_topic_;
        double mesh_period_ = 0.3;         // 增量块发布周期（秒）
        double mesh_full_period_ = 5.0;    // 全量重同步周期（秒）
        bool mesh_compress_ = true;        // payload zstd 压缩
        Vec3 sensor_origin_ = Vec3::Zero();// 传感器原点（base_link），用于法线定向
        uint32_t mesh_seq_ = 0;
        double last_mesh_full_ = -1e9;

        uint64_t frames_in_ = 0;
        uint64_t points_in_ = 0;
        uint64_t frames_dropped_ = 0;
    };

}  // namespace RusReconstruction
