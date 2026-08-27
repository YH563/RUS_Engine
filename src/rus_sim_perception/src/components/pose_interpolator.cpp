#include "components/pose_interpolator.hpp"

#include <algorithm>
#include <cmath>

namespace RusPerception {

    PoseInterpolator::PoseInterpolator(size_t max_cache) : max_cache_(max_cache) {}

    bool PoseInterpolator::Add(double stamp, const Eigen::Isometry3d& pose)
    {
        if (stamp <= Newest()) return false;  // 乱序 / 重复帧丢弃
        buffer_.push_back({stamp, pose});
        while (buffer_.size() > max_cache_) buffer_.pop_front();
        return true;
    }

    bool PoseInterpolator::Sample(double t, Eigen::Isometry3d& out, double* nearest_diff) const
    {
        if (buffer_.size() < 2) {
            if (nearest_diff) *nearest_diff = 0.0;
            return false;
        }
        if (t < buffer_.front().stamp || t > buffer_.back().stamp) {
            if (nearest_diff) {
                *nearest_diff = std::min(std::abs(t - buffer_.front().stamp),
                                         std::abs(t - buffer_.back().stamp));
            }
            return false;
        }

        // 精确命中端点：直接用端点位姿（upper_bound 无法插值到端点，需单独处理）
        if (t == buffer_.front().stamp) {
            if (nearest_diff) *nearest_diff = 0.0;
            out = buffer_.front().pose;
            return true;
        }
        if (t == buffer_.back().stamp) {
            if (nearest_diff) *nearest_diff = 0.0;
            out = buffer_.back().pose;
            return true;
        }

        // 第一个 stamp > t 的帧
        auto it = std::upper_bound(buffer_.begin(), buffer_.end(), t,
                                   [](double val, const TimedPose& p) { return val < p.stamp; });
        if (it == buffer_.begin() || it == buffer_.end()) {
            if (nearest_diff) *nearest_diff = 0.0;
            return false;  // 理论不可达（t 已排除两端点外）
        }
        const TimedPose& hi = *it;
        const TimedPose& lo = *(it - 1);

        const double w = (t - lo.stamp) / (hi.stamp - lo.stamp);
        if (w < 0.0 || w > 1.0) return false;

        if (nearest_diff) {
            *nearest_diff = std::min(std::abs(t - lo.stamp), std::abs(t - hi.stamp));
        }

        // 位置线性插值 + 姿态 SLERP
        const Eigen::Vector3d pos = lo.pose.translation() * (1.0 - w) + hi.pose.translation() * w;
        Eigen::Quaterniond q0(lo.pose.linear());
        Eigen::Quaterniond q1(hi.pose.linear());
        q0.normalize();
        q1.normalize();
        const Eigen::Quaterniond q = q0.slerp(w, q1);
        out = Eigen::Translation3d(pos) * q;
        return true;
    }

    void PoseInterpolator::Clear() { buffer_.clear(); }

}  // namespace RusPerception
