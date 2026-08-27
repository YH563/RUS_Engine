#pragma once

// ════════════════════════════════════════════════════════════════════
//  位姿时间插值器（components：公共能力）
//  ────────────────────────────────────────────────────────────────────
//  感知层统一的时间对齐组件：维护按时间戳排序的位姿滑窗，
//  对任意查询时刻 t 线性插值出位姿（位置 lerp + 姿态 SLERP）。
//  数据源：驱动状态（RobotState.timestamp + flange_pos），125Hz。
//  用途：点云帧按采集时间戳对齐到机械臂位姿；后续超声 / 图像
//        与位姿关联时复用同一组件。
//
//  纯算法实现：仅依赖 Eigen / std，无 ROS 类型，线程安全由调用方保证。
// ════════════════════════════════════════════════════════════════════

#include <cstddef>
#include <deque>

#include <Eigen/Geometry>

namespace RusPerception {

    /// 带时间戳的位姿（时间对齐缓冲单元）
    struct TimedPose {
        double stamp = 0.0;  // 生产端时间戳（秒，仿真时间）
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();  // 如 T_base_flange
    };

    /// 位姿时间插值器（纯算法）
    class PoseInterpolator {
    public:
        explicit PoseInterpolator(size_t max_cache = 256);

        /// 插入一帧位姿。要求 stamp 严格递增；乱序 / 重复被忽略并返回 false。
        bool Add(double stamp, const Eigen::Isometry3d& pose);

        /**
         * @brief 查询 t 时刻的位姿
         *
         * t 落在 [oldest, newest] 内时对前后两帧插值；
         * 越界返回 false（调用方应丢弃该帧，防止错位数据污染）。
         *
         * @param t            查询时刻（秒）
         * @param out          输出插值位姿
         * @param nearest_diff 可选输出：t 与最近一帧的时间差绝对值（时间对齐质量指标）
         * @return true 插值成功；false 位姿不足或 t 越界
         */
        bool Sample(double t, Eigen::Isometry3d& out, double* nearest_diff = nullptr) const;

        /**
         * @brief 取最新位姿（时间对齐降级用：真机相机/驱动时钟不同步时，
         *        用最近位姿近似，避免持续丢帧）。
         *
         * @return true 有缓存位姿
         */
        bool LatestPose(Eigen::Isometry3d& out) const {
            if (buffer_.empty()) return false;
            out = buffer_.back().pose;
            return true;
        }

        void Clear();

        size_t Size() const { return buffer_.size(); }
        double Oldest() const { return buffer_.empty() ? 0.0 : buffer_.front().stamp; }
        double Newest() const { return buffer_.empty() ? 0.0 : buffer_.back().stamp; }

    private:
        size_t max_cache_;
        std::deque<TimedPose> buffer_;  // 按 stamp 升序
    };

}  // namespace RusPerception
