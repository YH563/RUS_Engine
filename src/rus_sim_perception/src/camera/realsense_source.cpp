#include "camera/realsense_source.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <sstream>
#include <thread>
#include <utility>

#include <librealsense2/rs.hpp>

namespace RusPerception::Camera {
namespace {

    /// 读设备信息（个别型号缺字段 → 返回 fallback，不抛）
    std::string safe_info(const rs2::device& dev, rs2_camera_info key, const char* fallback)
    {
        try {
            return dev.get_info(key);
        } catch (const rs2::error&) {
            return fallback;
        }
    }

    /// 流是否已启用（未启用时 get_stream 抛异常）
    bool has_stream(const rs2::pipeline_profile& profile, rs2_stream type)
    {
        try {
            profile.get_stream(type);
            return true;
        } catch (const rs2::error&) {
            return false;
        }
    }


    std::string stream_desc(const rs2::pipeline_profile& profile, rs2_stream type)
    {
        try {
            const auto p = profile.get_stream(type).as<rs2::video_stream_profile>();
            std::ostringstream os;
            os << p.width() << "x" << p.height() << "@" << p.fps();
            return os.str();
        } catch (const rs2::error&) {
            return "无";
        }
    }

}  // namespace

    // ════════════════════════════════════════════════════════════════
    //  SDK 侧状态（pimpl：节点编译单元不引入 librealsense 头文件）
    // ════════════════════════════════════════════════════════════════
    struct RealSenseSource::Impl {
        RealSenseConfig cfg;

        rs2::pipeline pipe;
        std::unique_ptr<rs2::align> aligner;  // 按 align_to 创建；空 = 不对齐
        rs2::decimation_filter decimation;
        rs2::spatial_filter    spatial;
        rs2::temporal_filter   temporal;

        std::thread          thread;
        std::atomic<bool>    running{false};
        std::atomic<uint64_t> dropped{0};

        FrameCallback on_frame;
        TimeSource    now;
        LogSink       log;

        float       depth_scale = 0.001f;  // 深度单位（米/LSB），取自设备
        bool        use_color = false;     // 是否真的能取彩色（rgb 需 color 流 + 对齐）
        std::string frame_id;              // 点云所在光学系
        std::string device_info;           // "Intel RealSense D435 SN:xxx"
        std::string depth_desc, color_desc;
        uint32_t    seq = 0;

        void warn(const std::string& msg) const { if (log) log(msg); }
    };

    RealSenseSource::RealSenseSource(RealSenseConfig cfg) : impl_(std::make_unique<Impl>())
    {
        impl_->cfg = std::move(cfg);
    }

    RealSenseSource::~RealSenseSource()
    {
        Stop();
    }

    void RealSenseSource::Stop()
    {
        impl_->running.store(false);
        if (impl_->thread.joinable()) impl_->thread.join();
        try {
            impl_->pipe.stop();
        } catch (const rs2::error&) {
            // 未启动 / 已停止：忽略
        }
    }

    std::string RealSenseSource::Describe() const
    {
        const RealSenseConfig& cfg = impl_->cfg;
        std::ostringstream os;
        os << impl_->device_info
           << " 深度=" << impl_->depth_desc << " 彩色=" << impl_->color_desc
           << " 深度单位=" << impl_->depth_scale
           << " 对齐=" << cfg.align_to
           << " 颜色=" << (impl_->use_color ? "rgb" : "white")
           << " 抽稀=" << std::max(1, cfg.point_stride)
           << " 量程=[" << cfg.min_depth << "," << cfg.max_depth << "]m"
           << " 坐标系=" << impl_->frame_id;
        return os.str();
    }

    uint64_t RealSenseSource::Dropped() const
    {
        return impl_->dropped.load();
    }

    bool RealSenseSource::DeviceAvailable(std::string& detail)
    {
        try {
            rs2::context ctx;
            const rs2::device_list devs = ctx.query_devices();
            if (devs.size() == 0) {
                detail = "未检测到 RealSense 设备（USB 未接 / 权限不足 / 被其它进程独占）";
                return false;
            }
            std::ostringstream os;
            for (size_t i = 0; i < devs.size(); ++i) {
                if (i) os << ", ";
                os << safe_info(devs[i], RS2_CAMERA_INFO_NAME, "RealSense")
                   << " SN:" << safe_info(devs[i], RS2_CAMERA_INFO_SERIAL_NUMBER, "-");
            }
            detail = os.str();
            return true;
        } catch (const rs2::error& e) {
            detail = std::string("librealsense 查询设备失败: ") + e.what();
            return false;
        }
    }

    // ════════════════════════════════════════════════════════════════
    //  启动：开流 → 读设备参数 → 建对齐器 → 起采集线程
    // ════════════════════════════════════════════════════════════════
    bool RealSenseSource::Start(FrameCallback on_frame, TimeSource now, LogSink log, std::string& error)
    {
        if (impl_->running.load()) {
            error = "RealSense 源已在运行";
            return false;
        }
        impl_->on_frame = std::move(on_frame);
        impl_->now      = std::move(now);
        impl_->log      = std::move(log);

        const RealSenseConfig& cfg = impl_->cfg;
        const bool want_color = (cfg.color_mode == "rgb") || (cfg.align_to == "color");

        // 先探测设备：无设备时 librealsense 的 pipe.start() 会长时间阻塞且不给可读错误
        // （实测 >20s 无输出），提前用 context 查询 → 快速失败并给出可操作提示
        std::string probe;
        if (!DeviceAvailable(probe)) {
            error = std::string("RealSense 不可用：") + probe
                  + "（source=realsense 需要相机在位；无相机请改 source=ros_topic / replay）";
            return false;
        }
        impl_->device_info = probe;  // 供 Describe()；后面拿到 profile 后再补精确信息

        rs2::config conf;
        if (!cfg.serial.empty()) conf.enable_device(cfg.serial);
        conf.enable_stream(RS2_STREAM_DEPTH, cfg.width, cfg.height, RS2_FORMAT_Z16, cfg.fps);
        if (want_color) {
            conf.enable_stream(RS2_STREAM_COLOR, cfg.width, cfg.height, RS2_FORMAT_BGR8, cfg.fps);
        }

        rs2::pipeline_profile profile;
        try {
            profile = impl_->pipe.start(conf);
        } catch (const rs2::error& e) {
            // 各型号（D405 / L515 / D435i）支持的分辨率×帧率不同 → 退回设备默认配置
            impl_->warn(std::string("RealSense 指定配置启动失败（型号可能不支持该分辨率/帧率）: ")
                        + e.what() + "；退回设备默认配置");
            rs2::config fallback;
            if (!cfg.serial.empty()) fallback.enable_device(cfg.serial);
            fallback.enable_stream(RS2_STREAM_DEPTH);
            if (want_color) fallback.enable_stream(RS2_STREAM_COLOR);
            try {
                profile = impl_->pipe.start(fallback);
            } catch (const rs2::error& e2) {
                error = std::string("RealSense 启动失败: ") + e2.what();
                return false;
            }
        }

        const rs2::device dev = profile.get_device();
        impl_->device_info = safe_info(dev, RS2_CAMERA_INFO_NAME, "RealSense")
                           + " SN:" + safe_info(dev, RS2_CAMERA_INFO_SERIAL_NUMBER, "-");
        try {
            impl_->depth_scale = dev.first<rs2::depth_sensor>().get_depth_scale();
        } catch (const rs2::error& e) {
            impl_->depth_scale = 0.001f;
            impl_->warn(std::string("读取深度单位失败（用 0.001 兜底）: ") + e.what());
        }
        impl_->depth_desc = stream_desc(profile, RS2_STREAM_DEPTH);
        impl_->color_desc = stream_desc(profile, RS2_STREAM_COLOR);

        // 对齐器：color → 深度对齐到彩色（像素一一对应，可取 RGB）
        //         depth → 彩色对齐到深度（仅统一分辨率）
        if (cfg.align_to == "color" || cfg.align_to == "depth") {
            try {
                impl_->aligner = std::make_unique<rs2::align>(
                    cfg.align_to == "color" ? RS2_STREAM_COLOR : RS2_STREAM_DEPTH);
            } catch (const rs2::error& e) {
                impl_->aligner.reset();
                impl_->warn(std::string("创建对齐器失败，本次不做对齐: ") + e.what());
            }
        } else if (cfg.align_to != "none") {
            impl_->warn("未知 rs_align_to=" + cfg.align_to + "（可选 none/color/depth），按 none 处理");
        }

        // 颜色可用性：rgb 需「彩色流已启用」且「align_to=color」，否则降级为白点
        const bool has_color = has_stream(profile, RS2_STREAM_COLOR);
        impl_->use_color = (cfg.color_mode == "rgb") && has_color && (cfg.align_to == "color");
        if (cfg.color_mode == "rgb" && !impl_->use_color) {
            impl_->warn("rs_color_mode=rgb 需要彩色流可用且 rs_align_to=color（当前 align_to="
                        + cfg.align_to + "，彩色流=" + impl_->color_desc + "）→ 已降级为白点");
        }

        // 点云坐标系：对齐到彩色后按彩色内参反投影，属于彩色光学系
        impl_->frame_id = (cfg.align_to == "color") ? "camera_color_optical_frame"
                                                    : "camera_depth_optical_frame";

        impl_->running.store(true);
        impl_->thread = std::thread([this]() { capture_loop(); });
        return true;
    }

    // ════════════════════════════════════════════════════════════════
    //  采集线程：取帧集 → 对齐/滤波 → 反投影 → 上抛（唯一产出点）
    // ════════════════════════════════════════════════════════════════
    void RealSenseSource::capture_loop()
    {
        Impl& s = *impl_;
        const RealSenseConfig& cfg = s.cfg;

        // ⚠️ librealsense 的 wait_for_frames(timeout) 超时时**抛 rs2::error**
        //    （what() = "Frame didn't arrive within N"），并不返回空 frameset。
        //    故"超时"必须与"真·设备掉线"区分：pipe.start() 之后首帧要等深度/彩色
        //    两流建立 + align 同步（实测 0.2~1s），若一次超时就退出采集线程，会
        //    表现为「相机已连接、节点已启动，但 0Hz、地图恒空、前端/RViz 无数据」。
        constexpr int kFrameTimeoutMs         = 200;  // 一帧周期 66ms@15fps，余量充足
        constexpr int kMaxConsecutiveTimeouts = 50;   // 200ms×50 = 10s 无帧 → 判定掉线
        int consecutive_timeouts = 0;                 // 采集线程私有，无需原子

        while (s.running.load()) {
            rs2::frameset frames;
            try {
                frames = s.pipe.wait_for_frames(kFrameTimeoutMs);
            } catch (const rs2::error& e) {
                if (!s.running.load()) break;          // Stop() 引发的管道关闭
                // 超时（含首帧未就绪）不退出：计数后继续等，只有持续无帧才判掉线
                if (++consecutive_timeouts >= kMaxConsecutiveTimeouts) {
                    ++s.dropped;
                    s.warn("RealSense 连续取帧超时（约 "
                           + std::to_string(kMaxConsecutiveTimeouts * kFrameTimeoutMs / 1000)
                           + "s 无帧），判定设备掉线，采集线程退出: " + e.what());
                    break;
                }
                continue;
            }
            if (!s.running.load()) break;
            if (!frames) continue;                     // 防御：极少数版本返回空帧集

            consecutive_timeouts = 0;                  // 有帧 → 超时计数清零

            // 帧集 → 点云：对齐 → 可选滤波 → 内参反投影 + 量程过滤 → 颜色
            auto build = [&](rs2::frameset& fs, CloudRGB& out) -> bool {
                if (s.aligner) fs = s.aligner->process(fs);

                rs2::depth_frame depth = fs.get_depth_frame();
                if (!depth) return false;
                if (cfg.spatial_filter)  depth = s.spatial.process(depth).as<rs2::depth_frame>();
                if (cfg.temporal_filter) depth = s.temporal.process(depth).as<rs2::depth_frame>();
                if (cfg.decimation)      depth = s.decimation.process(depth).as<rs2::depth_frame>();

                const rs2::video_frame color = s.use_color
                    ? fs.get_color_frame() : rs2::video_frame(rs2::frame());

                const int w = static_cast<int>(depth.get_width());
                const int h = static_cast<int>(depth.get_height());
                if (w <= 0 || h <= 0) return false;

                // 反投影参数取自最终深度帧：对齐/抽稀会同步改变分辨率与内参
                float fx = 0.0f, fy = 0.0f, ppx = 0.0f, ppy = 0.0f;
                try {
                    const rs2_intrinsics intr =
                        depth.get_profile().as<rs2::video_stream_profile>().get_intrinsics();
                    fx = intr.fx; fy = intr.fy; ppx = intr.ppx; ppy = intr.ppy;
                } catch (const rs2::error&) {
                    return false;
                }
                if (fx <= 0.0f || fy <= 0.0f) return false;

                // 直接读 16 位原始深度缓冲（省去逐像素 get_distance() 的引用计数开销）
                const bool raw16 = (depth.get_bytes_per_pixel() == 2);
                const uint16_t* raw = raw16
                    ? reinterpret_cast<const uint16_t*>(depth.get_data()) : nullptr;
                const float scale = s.depth_scale;

                const uint8_t* cdata = nullptr;
                int cw = 0, ch = 0;
                if (color && color.get_bytes_per_pixel() == 3) {
                    cdata = static_cast<const uint8_t*>(color.get_data());
                    cw = static_cast<int>(color.get_width());
                    ch = static_cast<int>(color.get_height());
                }

                const int stride = std::max(1, cfg.point_stride);
                out.clear();
                out.reserve(static_cast<size_t>(w / stride + 1) * static_cast<size_t>(h / stride + 1));

                for (int y = 0; y < h; y += stride) {
                    for (int x = 0; x < w; x += stride) {
                        const size_t idx = static_cast<size_t>(y) * static_cast<size_t>(w)
                                         + static_cast<size_t>(x);
                        const float dist = raw16 ? (static_cast<float>(raw[idx]) * scale)
                                                 : depth.get_distance(x, y);
                        // 约束 2：0 = 无回波，超量程 = 无关区域 → 显式剔除（不产生 (0,0,0)）
                        if (dist < cfg.min_depth || dist > cfg.max_depth) continue;

                        pcl::PointXYZRGB p;
                        p.x = (static_cast<float>(x) - ppx) * dist / fx;
                        p.y = (static_cast<float>(y) - ppy) * dist / fy;
                        p.z = dist;  // 深度沿光轴：z = 深度值
                        if (cdata && x < cw && y < ch) {
                            const size_t o = (static_cast<size_t>(y) * static_cast<size_t>(cw)
                                              + static_cast<size_t>(x)) * 3;
                            p.b = cdata[o];      // RS2_FORMAT_BGR8
                            p.g = cdata[o + 1];
                            p.r = cdata[o + 2];
                        } else {
                            p.r = p.g = p.b = 200;  // 无彩色流：中性灰（非 0，避免全黑）
                        }
                        out.push_back(p);
                    }
                }
                return !out.empty();
            };

            auto cloud = std::make_shared<CloudRGB>();
            if (!build(frames, *cloud)) {  // 整帧无效（全被量程剔除 / 内参不可用）
                ++s.dropped;
                continue;
            }
            if (!s.on_frame) continue;

            CloudFrame frame;
            frame.cloud    = std::move(cloud);
            frame.stamp    = s.now ? s.now() : 0.0;  // 约束 1：ROS 时间，非设备时钟
            frame.seq      = ++s.seq;
            frame.frame_id = s.frame_id;
            s.on_frame(std::move(frame));
        }

        s.running.store(false);
    }

}  // namespace RusPerception::Camera
