#include "camera/source_factory.hpp"

#include <utility>

#include "camera/ros_topic_source.hpp"

namespace RusPerception::Camera {

    bool RealSenseAvailable(std::string& detail)
    {
        return RealSenseSource::DeviceAvailable(detail);
    }

    std::unique_ptr<IPointCloudSource> CreateSource(const SourceConfig& cfg,
                                                    rclcpp::Node* node,
                                                    std::string& error)
    {
        std::string type = cfg.type;
        if (type == "auto") {
            std::string detail;
            type = RealSenseAvailable(detail) ? "realsense" : "ros_topic";
        }

        if (type == "realsense") return std::make_unique<RealSenseSource>(cfg.rs);
        if (type == "ros_topic") return std::make_unique<RosTopicSource>(node, cfg.topic);
        if (type == "replay")    return std::make_unique<ReplaySource>(cfg.replay);

        error = "未知数据源类型 source=" + cfg.type + "（可选 auto / realsense / ros_topic / replay）";
        return nullptr;
    }

}  // namespace RusPerception::Camera
