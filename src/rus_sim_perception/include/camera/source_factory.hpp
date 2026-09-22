#pragma once

// ════════════════════════════════════════════════════════════════════
//  数据源工厂（camera：数据采集子模块）
//  ────────────────────────────────────────────────────────────────────
//  按 source 参数装配数据源，节点不关心具体实现：
//    auto       有 RealSense 设备 → realsense；否则 → ros_topic
//    realsense  RealSense 直连（librealsense2；设备独占，勿与 realsense2_camera 并存）
//    ros_topic  PointCloud2 话题（realsense2_camera / 仿真 / rosbag）
//    replay     离线 PCD 循环回放（无设备联调）
//
//  节点建议先自行调用 RealSenseAvailable() 以便把「为何选中该源」写进日志，
//  再把已解析的类型传给 CreateSource（避免重复探测设备）。
// ════════════════════════════════════════════════════════════════════

#include <memory>
#include <string>

#include "camera/point_cloud_source.hpp"
#include "camera/realsense_source.hpp"
#include "camera/replay_source.hpp"

namespace rclcpp { class Node; }  // 仅 RosTopicSource 需要，避免头文件引入 rclcpp

namespace RusPerception::Camera {

    /// 数据源总配置（由 perception_params.yaml 注入）
    struct SourceConfig {
        std::string type = "auto";  // source：auto / realsense / ros_topic / replay
        std::string topic;          // input_cloud_topic（ros_topic 用）
        RealSenseConfig rs;         // rs_*（realsense 用）
        ReplayConfig    replay;     // replay_*（replay 用）
    };

    /// RealSense 运行时是否可用（编译期支持 + 设备在位）；detail 填诊断信息
    bool RealSenseAvailable(std::string& detail);

    /**
     * @brief 创建数据源实例
     *
     * @param cfg   数据源配置
     * @param node  ROS 节点（仅 ros_topic 源需要，不持有所有权）
     * @param error 失败原因（type 非法时填写）
     * @return 数据源；type 非法返回 nullptr
     */
    std::unique_ptr<IPointCloudSource> CreateSource(const SourceConfig& cfg,
                                                    rclcpp::Node* node,
                                                    std::string& error);

}  // namespace RusPerception::Camera
