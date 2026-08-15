#pragma once

// ════════════════════════════════════════════════════════════════════
//  感知节点（ROS2 节点，仅通信与编排，不做算法）
//  ────────────────────────────────────────────────────────────────────
//  数据流（实时建图，无任务指令驱动）：
//    点云话题 ───────→ 缓存（最近一帧，生产者时间戳 header.stamp）
//    /driver/state ──→ PoseInterpolator（125Hz 位姿缓存 + 时间插值）
//    处理定时器 ─────→ 时间对齐 → 变换到 base_link → 滤波 → 增量入图
//                     → 双路发布：
//                       /preprocessed_cloud   raw（planning 规划输入）
//                       /sensor/pointcloud    压缩帧（SensorFrame → 前端）
//
//  对外指令（/perception/command）：仅 map_clear（清空地图，换场景用）
//
//  算法逻辑全部在独立模块（include/components、include/pointcloud），
//  节点只做订阅 / 调度 / 发布。
// ════════════════════════════════════════════════════════════════════

#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <rus_sim_interfaces/msg/robot_state.hpp>
#include <rus_sim_interfaces/msg/sensor_frame.hpp>
#include <rus_sim_interfaces/srv/command_service.hpp>

#include "components/pose_interpolator.hpp"
#include "components/sensor_encoder.hpp"
#include "pointcloud/cloud_filter_pipeline.hpp"
#include "pointcloud/map_manager.hpp"
#include "pointcloud/spatial_transformer.hpp"

namespace RusPerception {

    using PointCloud2 = sensor_msgs::msg::PointCloud2;
    using RobotStateMsg = rus_sim_interfaces::msg::RobotState;
    using CommandService = rus_sim_interfaces::srv::CommandService;
    using SensorFrame = rus_sim_interfaces::msg::SensorFrame;

    /// 感知节点（实时建图 + 传感器帧输出）
    class PerceptionNode : public rclcpp::Node {
    public:
        PerceptionNode();

    private:
        // ── 数据获取 ──
        void on_cloud(const PointCloud2::SharedPtr msg);
        void on_driver_state(const RobotStateMsg::SharedPtr msg);

        // ── 处理（定时器，独立回调组）──
        void process_frame();
        CloudRGBPtr load_cloud_from_file(const std::string& path);  // 加载 PCD → 双路发布，失败返回 nullptr
        std::string pcd_path_by_index(int idx) const;              // pcd_dir_ 下第 N 个 .pcd（字典序）
        bool publish_cloud(const CloudRGBPtr& cloud);              // 双路发布：raw + 压缩帧

        // ── 指令服务回调（/perception/command）──
        void handle_command(
            const std::shared_ptr<rmw_request_id_t> req_header,
            const std::shared_ptr<CommandService::Request> req,
            std::shared_ptr<CommandService::Response> res);

        // ── 参数 ──
        std::string input_cloud_topic_;
        std::string output_cloud_topic_;     // /preprocessed_cloud
        std::string sensor_cloud_topic_;     // /sensor/pointcloud
        std::string driver_state_topic_;
        double max_allowed_diff_sec_ = 0.05;  // 点云与位姿最大允许时间差（s）
        double process_period_ = 1.0;         // 处理周期（s）
        size_t max_pose_cache_ = 256;         // 位姿缓存上限
        size_t map_max_points_ = 0;           // 地图点数上限（0 = 不限）
        std::string input_pcd_;               // 启动即加载的 PCD 路径（空 = 不加载）
        std::string pcd_dir_;                 // load_cloud 按索引加载的 PCD 目录（空 = 仅 input_pcd_）

        // ── 缓存 ──
        PointCloud2::SharedPtr cloud_cache_;
        rclcpp::Time cloud_time_;             // 点云采集时刻（生产者时间戳）
        uint32_t sensor_seq_ = 0;             // SensorFrame 帧序号（每类型独立递增）

        // ── 算法模块 ──
        PoseInterpolator pose_interp_;
        PointCloud::SpatialTransformer transformer_;
        PointCloud::CloudFilterPipeline filter_;
        PointCloud::MapManager map_;
        SensorEncoder encoder_;

        // ── ROS 通信 ──
        rclcpp::Subscription<PointCloud2>::SharedPtr cloud_sub_;
        rclcpp::Subscription<RobotStateMsg>::SharedPtr state_sub_;
        rclcpp::Publisher<PointCloud2>::SharedPtr cloud_pub_;
        rclcpp::Publisher<SensorFrame>::SharedPtr sensor_pub_;
        rclcpp::Service<CommandService>::SharedPtr cmd_server_;
        rclcpp::TimerBase::SharedPtr process_timer_;
        rclcpp::CallbackGroup::SharedPtr process_cb_group_;
    };

}  // namespace RusPerception
