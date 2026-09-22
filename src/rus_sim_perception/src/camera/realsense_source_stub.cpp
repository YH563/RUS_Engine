// ════════════════════════════════════════════════════════════════════
//  RealSense 直连源——未编译版（stub）
//  ────────────────────────────────────────────────────────────────────
//  当 CMake 未找到 librealsense2（或 -DRUS_SIM_WITH_REALSENSE=OFF）时编译
//  本文件替代 realsense_source.cpp：保持同一套接口符号，节点代码无需任何
//  条件编译，source=realsense 在启动时报出可读的错误，其余数据源照常工作。
//
//  安装 librealsense2 后重新构建即可启用：sudo apt install librealsense2-dev
// ════════════════════════════════════════════════════════════════════

#include "camera/realsense_source.hpp"

#include <utility>

namespace RusPerception::Camera {

    /// 无 SDK：仅保留配置（Describe 用）
    struct RealSenseSource::Impl {
        RealSenseConfig cfg;
    };

    RealSenseSource::RealSenseSource(RealSenseConfig cfg) : impl_(std::make_unique<Impl>())
    {
        impl_->cfg = std::move(cfg);
    }

    RealSenseSource::~RealSenseSource() = default;

    bool RealSenseSource::Start(FrameCallback, TimeSource, LogSink log, std::string& error)
    {
        error = "本次构建未启用 RealSense 支持（缺 librealsense2）："
                "执行 sudo apt install librealsense2-dev 后重新 colcon build，"
                "或把 source 改为 ros_topic / replay";
        if (log) log(error);
        return false;
    }

    void RealSenseSource::Stop()
    {
    }

    std::string RealSenseSource::Describe() const
    {
        return "RealSense（未编译：缺 librealsense2 / RUS_SIM_WITH_REALSENSE=OFF）";
    }

    uint64_t RealSenseSource::Dropped() const
    {
        return 0;
    }

    bool RealSenseSource::DeviceAvailable(std::string& detail)
    {
        detail = "本次构建未启用 RealSense 支持（缺 librealsense2）";
        return false;
    }

}  // namespace RusPerception::Camera
