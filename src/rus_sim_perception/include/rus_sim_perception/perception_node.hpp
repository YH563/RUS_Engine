#pragma once

// ════════════════════════════════════════════════════════════════════
//  感知节点（ROS2 节点，仅通信与编排，不做算法）
//  ────────────────────────────────────────────────────────────────────
//  数据流（实时建图，无任务指令驱动）：
//    数据源 ─────────→ FrameSlot（最新一帧，覆盖式）
//      source=realsense  RealSense 直连（librealsense2，见 camera/realsense_source.hpp）
//      source=ros_topic  PointCloud2 话题（realsense2_camera / 仿真 / rosbag）
//      source=replay     离线 PCD 循环回放（无设备联调，走同一条处理链）
//    /driver/state ──→ PoseInterpolator（125Hz 位姿缓存 + 时间插值）
//    处理定时器 ─────→ 时间对齐 → 变换到 base_link → 滤波 → 入图（mapping_mode）
//                     → 双路发布：
//                       /preprocessed_cloud   地图快照（rolling/accumulate）或当前帧（none）
//                       /perception/frame     当前帧（RViz 实时可视化）
//                       /sensor/pointcloud    压缩帧（SensorFrame → 前端 /sensor 通路）
//
//  对外指令（/perception/command）：仅 map_clear（清空地图，换场景用）
//
//  算法/采集逻辑全部在独立模块（include/camera、include/components、
//  include/pointcloud），节点只做数据源装配 / 调度 / 发布。
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

#include "camera/frame_slot.hpp"
#include "camera/point_cloud_source.hpp"
#include "camera/source_factory.hpp"
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
        ~PerceptionNode() override;  // 停数据源（含采集线程 join）

    private:
        // ── 数据获取 ──
        void on_driver_state(const RobotStateMsg::SharedPtr msg);
        void on_cloud_frame(Camera::CloudFrame&& frame);  // 数据源回调 → 帧槽（任意线程）

        // ── 处理（定时器，独立回调组）──
        void process_frame();
        bool publish_frame(const CloudRGBPtr& cloud, double stamp);  // 实时帧发布（/perception/frame）
        CloudRGBPtr load_cloud_from_file(const std::string& path);   // 加载 PCD → 直发一次，失败返回 nullptr
        std::string pcd_path_by_index(int idx) const;                // pcd_dir_ 下第 N 个 .pcd（字典序）
        bool publish_cloud(const CloudRGBPtr& cloud, double stamp);  // 地图快照/当前帧发布：raw + 压缩帧
        bool mapping_enabled() const { return mapping_mode_ != "none"; }

        // ── 指令服务回调（/perception/command）──
        void handle_command(
            const std::shared_ptr<rmw_request_id_t> req_header,
            const std::shared_ptr<CommandService::Request> req,
            std::shared_ptr<CommandService::Response> res);

        // ── 参数 ──
        std::string source_type_;             // 数据源：auto / realsense / ros_topic / replay
        std::string input_cloud_topic_;       // ros_topic 源的输入话题
        std::string output_cloud_topic_;      // /preprocessed_cloud（planning 输入）
        std::string frame_topic_;             // /perception/frame（当前帧，实时可视化）
        std::string sensor_cloud_topic_;      // /sensor/pointcloud
        std::string driver_state_topic_;
        std::string mapping_mode_;            // none（不建图）/ rolling（上限降采样）/ accumulate（只累积）
        Camera::SourceConfig source_cfg_;     // 数据源配置（rs_* / replay_* 参数）
        double max_allowed_diff_sec_ = 0.05;  // 点云与位姿最大允许时间差（s）
        double process_period_ = 0.1;         // 处理周期（s）：对齐→变换→滤波→入图
        double map_publish_period_ = 2.0;     // 地图发布周期（s）：仅 rolling/accumulate 生效
        bool allow_stale_pose_ = false;       // 位姿对齐失败时是否降级用最近位姿（真机时钟不同步用）
        size_t max_pose_cache_ = 256;         // 位姿缓存上限
        size_t map_max_points_ = 500000;      // 地图点数上限（0 = 不限；仅 rolling 生效）
        std::string input_pcd_;               // 启动即加载的 PCD 路径（空 = 不加载）
        std::string pcd_dir_;                 // load_cloud 按索引加载的 PCD 目录（空 = 仅 input_pcd_）

        // ── 数据源 / 帧槽 ──
        std::unique_ptr<Camera::IPointCloudSource> source_;  // 由 source 参数装配（启动时选定）
        Camera::FrameSlot slot_;                             // 最新一帧（采集线程 → 处理线程）
        uint32_t sensor_seq_ = 0;             // SensorFrame 帧序号（每类型独立递增）

        // ── 处理状态（防重复入图 / 地图节流发布 / 帧率统计）──
        double last_processed_stamp_ = -1.0;  // 已处理过的点云时间戳（防同一帧反复入图）
        double last_map_publish_time_ = 0.0;  // 上次发布地图的时刻（wall clock）
        size_t processed_frames_ = 0;         // 已处理帧计数（帧率统计）
        size_t dropped_frames_ = 0;           // 丢弃帧计数（位姿对齐失败 / 空帧）
        double process_rate_hz_ = 0.0;        // 实际处理频率（统计，Hz）
        double last_process_log_time_ = 0.0;  // 上次统计日志时刻（wall clock）

        // ── 算法模块 ──
        PoseInterpolator pose_interp_;
        PointCloud::SpatialTransformer transformer_;
        PointCloud::CloudFilterPipeline filter_;
        PointCloud::MapManager map_;
        SensorEncoder encoder_;

        // ── ROS 通信 ──
        rclcpp::Subscription<RobotStateMsg>::SharedPtr state_sub_;
        rclcpp::Publisher<PointCloud2>::SharedPtr cloud_pub_;   // 地图快照 / 当前帧（planning 输入）
        rclcpp::Publisher<PointCloud2>::SharedPtr frame_pub_;   // 当前帧（RViz 可视化）
        rclcpp::Publisher<SensorFrame>::SharedPtr sensor_pub_;
        rclcpp::Service<CommandService>::SharedPtr cmd_server_;
        rclcpp::TimerBase::SharedPtr process_timer_;
        rclcpp::CallbackGroup::SharedPtr process_cb_group_;
    };

}  // namespace RusPerception
