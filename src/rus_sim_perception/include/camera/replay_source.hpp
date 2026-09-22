#pragma once

// ════════════════════════════════════════════════════════════════════
//  离线回放点云源（camera：数据采集子模块）
//  ────────────────────────────────────────────────────────────────────
//  把 PCD 文件当作「相机」按固定频率反复推送，用于无设备联调：
//  走的是与真实相机完全相同的路径（帧槽 → 时间对齐 → 标定变换 → 滤波
//  → 建图 → 双路发布），因此可验证标定矩阵与整条实时管线，
//  也能复现「发布频率 × 点数」的负载。
//
//  ⚠ 语义差别（勿混用）：
//    - 本源产出的是【相机光学系】点云（需位姿 + 标定矩阵才能到 base_link）；
//    - `input_pcd` / `load_cloud` 加载的 PCD 视为【base_link】系场景
//      （不做坐标变换，直接入图，用于有场景无相机的场合）。
//
//  依赖：仅 PCL / std（时间戳由 TimeSource 提供）。
// ════════════════════════════════════════════════════════════════════

#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "camera/point_cloud_source.hpp"

namespace RusPerception::Camera {

    /// 回放配置（由 perception_params.yaml 注入）
    struct ReplayConfig {
        std::string path;       // PCD 文件，或目录（目录内 *.pcd 按字典序）
        double fps = 5.0;       // 回放频率（Hz）
        bool   loop = true;     // 循环回放（false = 跑完一遍停止推送）
        std::string frame_id = "camera_optical_frame";  // 诊断用坐标系名
    };

    /// 离线 PCD 回放源
    class ReplaySource : public IPointCloudSource {
    public:
        explicit ReplaySource(ReplayConfig cfg);

        bool Start(FrameCallback on_frame, TimeSource now, LogSink log, std::string& error) override;
        void Stop() override;
        std::string Name() const override { return "replay"; }
        std::string Describe() const override;
        uint64_t Dropped() const override { return 0; }

    private:
        void loop();

        ReplayConfig cfg_;
        std::vector<CloudRGBPtr> clouds_;   // 预加载（共享指针，每帧不拷贝）
        FrameCallback on_frame_;
        TimeSource    now_;
        LogSink       log_;
        std::thread   thread_;
        std::atomic<bool> running_{false};
        uint32_t seq_ = 0;
    };

}  // namespace RusPerception::Camera
