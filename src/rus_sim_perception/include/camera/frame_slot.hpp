#pragma once

// ════════════════════════════════════════════════════════════════════
//  最新帧单槽（camera：数据采集子模块）
//  ────────────────────────────────────────────────────────────────────
//  数据源线程 → 处理定时器线程 的唯一交接点。
//
//  语义：覆盖式只留最新一帧（与 /sensor、/state 的「可丢帧」语义一致）。
//    生产端（采集线程/订阅回调）Push 永不阻塞；
//    消费端（处理定时器）TakeNewerThan 取走并消费，按时间戳去重。
//  处理线程比采集慢时的取舍：宁可丢弃中间帧，也不排队积压
//  （点云是「当前场景快照」，旧帧对实时建图无意义，且排队会无限吃内存）。
//
//  依赖：仅 PCL / std，无 ROS 类型。
// ════════════════════════════════════════════════════════════════════

#include <cstdint>
#include <mutex>
#include <string>

#include "camera/point_cloud_source.hpp"

namespace RusPerception::Camera {

    /// 最新帧单槽（线程安全）
    class FrameSlot {
    public:
        /// 生产者：覆盖写入最新一帧（采集线程调用，非阻塞）
        void Push(CloudFrame frame);

        /**
         * @brief 消费者：取比 since 更新的一帧（处理线程调用）
         *
         * 弹出槽内容：比 since 新 → 写入 out 返回 true；
         * 否则（重复帧 / 旧帧）直接丢弃返回 false。
         *
         * @param since 已处理过的最大时间戳（秒），首次传 -1
         * @param out   输出帧（返回 true 时有效）
         */
        bool TakeNewerThan(double since, CloudFrame& out);

        /// 是否已有未消费的帧
        bool Empty() const;

        /// 清空（换场景 / 重置）
        void Clear();

        uint64_t Pushed() const;       ///< 累计推入帧数
        uint64_t Overwritten() const;  ///< 被覆盖丢弃的帧数（处理跟不上采集）
        uint64_t Stale() const;        ///< 因时间戳不新被丢弃的帧数（重复帧）

    private:
        mutable std::mutex mtx_;
        CloudFrame frame_;
        bool     valid_ = false;
        uint64_t pushed_ = 0;
        uint64_t overwritten_ = 0;
        uint64_t stale_ = 0;
    };

}  // namespace RusPerception::Camera
