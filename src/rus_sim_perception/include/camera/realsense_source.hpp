#pragma once

// ════════════════════════════════════════════════════════════════════
//  RealSense 直连点云源（camera：数据采集子模块）
//  ────────────────────────────────────────────────────────────────────
//  不经 realsense2_camera，直接调 librealsense2（本机 2.57.7）：
//    设备 → 深度/彩色流 → 可选对齐/滤波 → 反投影为彩色点云 → 上抛帧
//  优点：省一次进程间拷贝与 ROS 序列化，无包装节点依赖，可精确控制
//        抽稀/量程/对齐；代价：与 ROS 生态（TF / 相机内参话题）解耦。
//
//  三条硬约束（违反会静默出错，勿改）：
//    1. 时间戳用 node->now()（经 TimeSource 注入）：RealSense SDK 的
//       设备时钟与 ROS 时钟不同源，直接拿设备时间会与 /driver/state 的
//       位姿时间轴不可比，时间对齐必然失败。
//    2. 无效深度（0 = 无回波）与超量程点显式剔除，不得产出 (0,0,0)——
//       否则会在 base_link 原点堆出一团"鬼点"入图。
//    3. 点云坐标系由 rs_align_to 决定：
//         color → camera_color_optical_frame
//         none/depth → camera_depth_optical_frame
//       camera_to_flange 标定矩阵必须与之一致。旧标定取自
//       `/camera/camera/depth/color/points`（= 彩色光学系），故默认
//       align_to=color 的老配置需用 color 才继续成立。
//
//  设备独占：同一台相机不能被 librealsense2 与本源之外的 realsense2_camera
//  同时打开（会报 "Device or resource busy"）。要并存请把 source 改为
//  ros_topic（走包装节点），两者不要同时开。
//
//  编译开关：RUS_SIM_WITH_REALSENSE（缺 librealsense2 时 CMake 自动
//  退化为 realsense_source_stub.cpp，节点与其余数据源照常可用）。
// ════════════════════════════════════════════════════════════════════

#include <memory>
#include <string>

#include "camera/point_cloud_source.hpp"

namespace RusPerception::Camera {

    /// RealSense 采集配置（对应 perception_params.yaml 的 rs_* 参数）
    struct RealSenseConfig {
        std::string serial;                // rs_serial：设备序列号（空 = 第一台可用设备）
        int  width  = 640;                 // rs_width：深度/彩色流宽
        int  height = 480;                 // rs_height：深度/彩色流高
        int  fps    = 15;                  // rs_fps：帧率
        std::string align_to = "none";     // rs_align_to：none | color | depth（决定点云坐标系）
        std::string color_mode = "rgb";    // rs_color_mode：rgb | white（rgb 需 align_to=color）
        int  point_stride = 2;             // rs_point_stride：像素抽稀步长（1 = 全分辨率）
        float min_depth = 0.15f;           // rs_min_depth：有效深度下限（米）
        float max_depth = 2.0f;            // rs_max_depth：有效深度上限（米）
        bool decimation = false;           // rs_decimation：SDK 抽稀滤波（降分辨率采样）
        bool spatial_filter = false;       // rs_spatial_filter：空间滤波（深度降噪，平整表面）
        bool temporal_filter = false;      // rs_temporal_filter：时间滤波（静止场景降噪）
    };

    /// RealSense 直连点云源（pimpl 隔离 SDK 头文件，头文件不引入 librealsense）
    class RealSenseSource : public IPointCloudSource {
    public:
        explicit RealSenseSource(RealSenseConfig cfg);
        ~RealSenseSource() override;

        bool Start(FrameCallback on_frame, TimeSource now, LogSink log, std::string& error) override;
        void Stop() override;
        std::string Name() const override { return "realsense"; }
        std::string Describe() const override;
        uint64_t Dropped() const override;

        /// 当前环境是否有可用设备（编译期支持 + 设备在位）；detail 填诊断信息
        static bool DeviceAvailable(std::string& detail);

    private:
        void capture_loop();  // 采集线程主体：wait_for_frames → 反投影 → 上抛

        struct Impl;
        std::unique_ptr<Impl> impl_;
    };

}  // namespace RusPerception::Camera
