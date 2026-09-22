#include "camera/ros_topic_source.hpp"

#include <exception>
#include <utility>

#include <pcl_conversions/pcl_conversions.h>

namespace RusPerception::Camera {

    RosTopicSource::RosTopicSource(rclcpp::Node* node, std::string topic)
        : node_(node), topic_(std::move(topic))
    {
    }

    bool RosTopicSource::Start(FrameCallback on_frame, TimeSource now, LogSink log, std::string& error)
    {
        if (!node_) {
            error = "RosTopicSource 需要 ROS 节点上下文";
            return false;
        }
        on_frame_ = std::move(on_frame);
        now_      = std::move(now);
        log_      = std::move(log);

        // 队列深度 10：与旧实现一致（点云大，积压无意义，槽位另有覆盖式去抖）
        sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
            topic_, 10,
            [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { on_msg(msg); });
        return true;
    }

    void RosTopicSource::Stop()
    {
        sub_.reset();
    }

    std::string RosTopicSource::Describe() const
    {
        return "ROS 话题 " + topic_ + "（PointCloud2 → 相机光学系点云）";
    }

    void RosTopicSource::on_msg(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (!msg || msg->data.empty() || !on_frame_) {
            ++dropped_;
            return;
        }

        auto cloud = std::make_shared<CloudRGB>();
        try {
            pcl::fromROSMsg(*msg, *cloud);
        } catch (const std::exception& e) {
            // 字段布局不兼容（如 XYZI / 无 rgb 字段）等：丢弃并节流告警
            const uint64_t n = ++dropped_;
            if (log_ && (n == 1 || n % 50 == 0)) {
                log_("点云消息解析失败（累计 " + std::to_string(n) + " 次）: " + e.what());
            }
            return;
        }
        if (cloud->empty()) {
            ++dropped_;
            return;
        }

        CloudFrame frame;
        frame.cloud = std::move(cloud);
        // 采集时刻用生产者 header.stamp；驱动未填时间戳（全零）时退化为到达时刻
        const bool zero_stamp = (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0);
        frame.stamp    = zero_stamp ? (now_ ? now_() : 0.0)
                                    : rclcpp::Time(msg->header.stamp).seconds();
        frame.seq      = ++seq_;
        frame.frame_id = msg->header.frame_id.empty() ? topic_ : msg->header.frame_id;
        on_frame_(std::move(frame));
    }

}  // namespace RusPerception::Camera
