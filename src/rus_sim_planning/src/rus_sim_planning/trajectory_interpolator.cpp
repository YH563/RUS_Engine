#include "rus_sim_planning/trajectory_interpolator.hpp"

namespace RusSimPlanning {

    // ---- SetWaypoints — 设置稀疏路径点 ----
    void TrajectoryInterpolator::SetWaypoints(const Trajectory& waypoints)
    {
        waypoints_ = waypoints;
        dense_.clear();
        idx_ = 0;
    }

    // ---- SetRobotState — 刷新机械臂状态 ----
    void TrajectoryInterpolator::SetRobotState(const RusUtils::RobotState& state)
    {
        robot_state_ = state;
    }

    // ---- Interpolate — 插值稠密化：相邻路径点间插入中间点 ----
    bool TrajectoryInterpolator::Interpolate(int points_per_segment)
    {
        dense_.clear();
        idx_ = 0;

        if (waypoints_.size() < 2) {
            dense_ = waypoints_;  // 不足两段时原样输出
            return waypoints_.size() >= 2;
        }
        if (points_per_segment < 1) points_per_segment = 1;

        // 相邻路径点间插值
        for (std::size_t i = 0; i + 1 < waypoints_.size(); ++i) {
            const auto& a = waypoints_[i];
            const auto& b = waypoints_[i + 1];
            for (int k = 0; k < points_per_segment; ++k) {
                double t = static_cast<double>(k) / static_cast<double>(points_per_segment);
                dense_.push_back(lerp_pose(a, b, t));
            }
        }
        // 末尾补上最后一个路径点
        dense_.push_back(waypoints_.back());
        return true;
    }

    // ---- NextTarget — 推进输出：取当前目标并推进到下一个 ----
    std::optional<geometry_msgs::msg::Pose> TrajectoryInterpolator::NextTarget()
    {
        if (idx_ >= dense_.size())
            return std::nullopt;
        return dense_[idx_++];
    }

    // ---- CurrentTarget — 当前目标（不推进） ----
    std::optional<geometry_msgs::msg::Pose> TrajectoryInterpolator::CurrentTarget() const
    {
        if (idx_ >= dense_.size())
            return std::nullopt;
        return dense_[idx_];
    }

    // ---- IsFinished — 是否已全部推进完 ----
    bool TrajectoryInterpolator::IsFinished() const
    {
        return dense_.empty() || idx_ >= dense_.size();
    }

    // ---- Reset — 复位：清空轨迹与索引 ----
    void TrajectoryInterpolator::Reset()
    {
        waypoints_.clear();
        dense_.clear();
        idx_ = 0;
    }

    // ---- lerp_pose — 两路径点间插值：位置线性 + 姿态 slerp ----
    geometry_msgs::msg::Pose TrajectoryInterpolator::lerp_pose(
        const geometry_msgs::msg::Pose& a,
        const geometry_msgs::msg::Pose& b,
        double t)
    {
        geometry_msgs::msg::Pose out;
        // 位置线性插值
        out.position.x = a.position.x + (b.position.x - a.position.x) * t;
        out.position.y = a.position.y + (b.position.y - a.position.y) * t;
        out.position.z = a.position.z + (b.position.z - a.position.z) * t;

        // 姿态球面插值（slerp）
        Eigen::Quaterniond qa(a.orientation.w, a.orientation.x, a.orientation.y, a.orientation.z);
        Eigen::Quaterniond qb(b.orientation.w, b.orientation.x, b.orientation.y, b.orientation.z);
        Eigen::Quaterniond q = qa.slerp(t, qb);
        out.orientation.w = q.w();
        out.orientation.x = q.x();
        out.orientation.y = q.y();
        out.orientation.z = q.z();
        return out;
    }

}  // namespace RusSimPlanning
