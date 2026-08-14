#include "rus_sim_planning/collision_checker.hpp"

#include <limits>

namespace RusSimPlanning {

    // ---- SetCloud — 设置参考点云并构建 KDTree ----
    void CollisionChecker::SetCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
    {
        cloud_ = cloud;
        if (!cloud_ || cloud_->empty()) {
            cloud_ready_ = false;
            return;
        }
        tree_.setInputCloud(cloud_);
        cloud_ready_ = true;
    }

    // ---- MinDistanceToCloud — 点到点云最小距离 ----
    double CollisionChecker::MinDistanceToCloud(const Eigen::Vector3d& p) const
    {
        if (!cloud_ready_)
            return -1.0;
        pcl::PointXYZ query;
        query.x = p.x();
        query.y = p.y();
        query.z = p.z();
        std::vector<int> indices(1);
        std::vector<float> sqr_dists(1);
        tree_.nearestKSearch(query, 1, indices, sqr_dists);
        if (indices.empty() || indices[0] < 0)
            return std::numeric_limits<double>::max();
        return std::sqrt(sqr_dists[0]);
    }

    // ---- CheckPose — 单点位姿安全检查 ----
    bool CollisionChecker::CheckPose(const geometry_msgs::msg::Pose& pose, double min_clearance) const
    {
        if (!cloud_ready_)
            return false;  // 无点云无法校验 → 视为不安全
        Eigen::Vector3d p(pose.position.x, pose.position.y, pose.position.z);
        return MinDistanceToCloud(p) >= min_clearance;
    }

    // ---- CheckTrajectory — 整条轨迹安全检查 ----
    bool CollisionChecker::CheckTrajectory(const Trajectory& trajectory, double min_clearance) const
    {
        if (trajectory.empty())
            return false;
        for (const auto& pose : trajectory) {
            if (!CheckPose(pose, min_clearance))
                return false;
        }
        return true;
    }

    // ---- Reset — 复位：清空点云 ----
    void CollisionChecker::Reset()
    {
        cloud_.reset();
        cloud_ready_ = false;
    }

}  // namespace RusSimPlanning
