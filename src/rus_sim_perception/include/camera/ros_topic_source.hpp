#pragma once

// ════════════════════════════════════════════════════════════════════
//  ROS 话题点云源（camera：数据采集子模块）
//  ────────────────────────────────────────────────────────────────────
//  订阅 PointCloud2 话题 → 转 PCL 点云 → 上抛帧。
//  适用：realsense2_camera 包装节点、仿真器、任何 PointCloud2 发布者。
//
//  与 RealSense 直连的差别：本源不打开设备，多进程 + 一帧拷贝开销，
//  但可与 realsense2_camera 的其它话题（彩色图 / 内参）共存，
//  且支持 rosbag 回放与仿真器（设备无关）。
//
//  时间戳取生产者 header.stamp（采集时刻，非到达时刻）；若驱动未填时间戳
//  （全零）则退化为 TimeSource 的当前时刻，避免时间对齐必然失败。
// ════════════════════════════════════════════════════════════════════

#include <atomic>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "camera/point_cloud_source.hpp"

namespace RusPerception::Camera {

    /// ROS 话题点云源
    class RosTopicSource : public IPointCloudSource {
    public:
        RosTopicSource(rclcpp::Node* node, std::string topic);

        bool Start(FrameCallback on_frame, TimeSource now, LogSink log, std::string& error) override;
        void Stop() override;
        std::string Name() const override { return "ros_topic"; }
        std::string Describe() const override;
        uint64_t Dropped() const override { return dropped_.load(); }

    private:
        void on_msg(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

        rclcpp::Node* node_;                  // 订阅挂在节点上（不持有）
        std::string   topic_;
        FrameCallback on_frame_;
        TimeSource    now_;
        LogSink       log_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
        std::atomic<uint64_t> dropped_{0};    // 解析失败 / 空消息
        uint32_t seq_ = 0;
    };

}  // namespace RusPerception::Camera
