#include "camera/replay_source.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <utility>

#include "pointcloud/cloud_io.hpp"

namespace RusPerception::Camera {

    ReplaySource::ReplaySource(ReplayConfig cfg) : cfg_(std::move(cfg))
    {
    }

    bool ReplaySource::Start(FrameCallback on_frame, TimeSource now, LogSink log, std::string& error)
    {
        if (cfg_.path.empty()) {
            error = "回放源未配置 replay_path";
            return false;
        }
        on_frame_ = std::move(on_frame);
        now_      = std::move(now);
        log_      = std::move(log);

        // ── 收集 PCD（单文件或目录内 *.pcd，字典序）──
        std::vector<std::string> files;
        std::error_code ec;
        if (std::filesystem::is_directory(cfg_.path, ec)) {
            for (const auto& entry : std::filesystem::directory_iterator(cfg_.path)) {
                if (entry.is_regular_file() && entry.path().extension() == ".pcd") {
                    files.push_back(entry.path().string());
                }
            }
            std::sort(files.begin(), files.end());
        } else {
            files.push_back(cfg_.path);
        }
        if (files.empty()) {
            error = "回放路径下无 .pcd 文件: " + cfg_.path;
            return false;
        }

        // ── 预加载（一次性读入内存，回放期间零 IO）──
        for (const std::string& f : files) {
            auto cloud = std::make_shared<CloudRGB>();
            if (!PointCloud::LoadPcd(f, *cloud) || cloud->empty()) {
                if (log_) log_("回放点云加载失败，已跳过: " + f);
                continue;
            }
            clouds_.push_back(std::move(cloud));
        }
        if (clouds_.empty()) {
            error = "回放点云全部加载失败: " + cfg_.path;
            return false;
        }

        running_.store(true);
        thread_ = std::thread([this]() { loop(); });
        return true;
    }

    void ReplaySource::Stop()
    {
        running_.store(false);
        if (thread_.joinable()) thread_.join();
    }

    std::string ReplaySource::Describe() const
    {
        char buf[128];
        std::snprintf(buf, sizeof(buf), "离线回放 %s（%zu 帧, %.1f Hz, %s）",
                      cfg_.path.c_str(), clouds_.size(), cfg_.fps,
                      cfg_.loop ? "循环" : "单趟");
        return buf;
    }

    void ReplaySource::loop()
    {
        // 固定周期推送；等待分片进行，保证 Stop() 能在 ~5ms 内返回
        const double period = 1.0 / std::max(0.1, cfg_.fps);
        const auto   period_d = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(period));
        auto next = std::chrono::steady_clock::now();

        size_t idx = 0;
        while (running_.load()) {
            next += period_d;
            while (running_.load() && std::chrono::steady_clock::now() < next) {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
            if (!running_.load() || !on_frame_) break;

            CloudFrame frame;
            frame.cloud    = clouds_[idx];        // 共享指针：处理链只读该点云（Transform 另分配）
            frame.stamp    = now_ ? now_() : 0.0; // 采集时刻 = 推送时刻（ROS 时间）
            frame.seq      = ++seq_;
            frame.frame_id = cfg_.frame_id;
            on_frame_(std::move(frame));

            if (++idx >= clouds_.size()) {
                if (!cfg_.loop) break;
                idx = 0;
            }
        }
        running_.store(false);
    }

}  // namespace RusPerception::Camera
