#pragma once

// ════════════════════════════════════════════════════════════════════
//  点云数据源抽象（camera：数据采集子模块）
//  ────────────────────────────────────────────────────────────────────
//  感知层唯一数据入口：把「RealSense 直连 / ROS 话题 / 离线回放」统一成
//  「产出相机光学系点云 + 采集时刻」，由 source 参数在启动时选定
//  （见 source_factory.hpp）。
//
//  实现契约（各源必须遵守，违反会导致下游静默出错）：
//    1. 坐标系：点云为相机光学系（x 右 / y 下 / z 前），源不做任何坐标变换
//       —— 相机→base_link 变换只由 pointcloud/SpatialTransformer 负责，
//       标定矩阵只有一份，RealSense 直连与 realsense2_camera 话题共用。
//    2. 时间戳：必须是 ROS 时间（秒）。RealSense SDK 的设备时钟与 ROS 时钟
//       不同源，必须用传入的 TimeSource（= node->now()）取时刻，否则与
//       /driver/state 的位姿时间轴不可比，时间对齐必然失败。
//    3. 无效点显式剔除：深度 0 / NaN / 超量程的点在源内丢弃，不得以
//       (0,0,0) 形式进入下游（否则会在 base_link 原点堆出一团鬼点）。
//    4. 回调线程：帧回调可能来自独立采集线程（RealSense / 回放）或 ROS
//       执行器线程（话题订阅），消费端必须线程安全（见 frame_slot.hpp）。
//
//  依赖：仅 PCL / std；ROS 适配见 ros_topic_source.hpp。
// ════════════════════════════════════════════════════════════════════

#include <cstdint>
#include <functional>
#include <string>

#include "components/types.hpp"

namespace RusPerception::Camera {

    /// 采集侧时钟：返回 ROS 时间（秒）
    using TimeSource = std::function<double()>;

    /// 采集侧日志（源无 ROS 依赖，日志经此回调上抛给节点统一打印）
    using LogSink = std::function<void(const std::string&)>;

    /// 一帧点云（跨数据源统一表示）
    struct CloudFrame {
        CloudRGBPtr cloud;        // 相机光学系点云（未做任何变换）
        double      stamp = 0.0;  // 采集时刻（ROS 时间，秒）
        uint32_t    seq   = 0;    // 源内帧序号（单调递增，丢帧诊断用）
        std::string frame_id;     // 源坐标系名（诊断用，如 camera_color_optical_frame）
    };

    /// 帧回调（在源线程内被调用，实现方需保证线程安全）
    using FrameCallback = std::function<void(CloudFrame&&)>;

    /// 点云数据源接口
    class IPointCloudSource {
    public:
        virtual ~IPointCloudSource() = default;

        /**
         * @brief 启动采集（打开设备 / 建立订阅 / 起采集线程）
         *
         * @param on_frame 帧回调（可能来自独立线程）
         * @param now      采集侧时钟（ROS 时间），源自身无 ROS 时间时使用
         * @param log      日志上抛（WARN 级），可为空
         * @param error    失败原因（返回 false 时填写）
         * @return true 启动成功
         */
        virtual bool Start(FrameCallback on_frame, TimeSource now, LogSink log, std::string& error) = 0;

        /// 停止采集并回收资源（幂等）
        virtual void Stop() = 0;

        /// 源类型标识："realsense" / "ros_topic" / "replay"
        virtual std::string Name() const = 0;

        /// 运行状态描述（设备型号 / 话题 / 文件），启动日志用
        virtual std::string Describe() const = 0;

        /// 因内部错误丢弃的帧数（0 = 正常）
        virtual uint64_t Dropped() const { return 0; }
    };

}  // namespace RusPerception::Camera
