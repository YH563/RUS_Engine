#pragma once

// ════════════════════════════════════════════════════════════════════
//  碰撞检查模块
//  ────────────────────────────────────────────────────────────────────
//  基于参考点云构建 KDTree，通过最近邻距离检查轨迹/位姿是否安全：
//    - CheckPose：      单个位姿到点云的最小距离 ≥ 安全间隙
//    - CheckTrajectory：整条轨迹逐点校验
//  ⚠️ 当前暂未接入规划流程（超声扫查需接触，安全间隙语义待重新设计），
//     后续若用 MuJoCo 做自碰撞 + 点云碰撞可替换本模块实现。
// ════════════════════════════════════════════════════════════════════

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <geometry_msgs/msg/pose.hpp>

#include "rus_sim_planning/trajectory_generator.hpp"

namespace RusSimPlanning {

    /**
     * @brief 碰撞检查模块
     *
     * 基于参考点云构建 KDTree，通过最近邻距离检查轨迹/位姿是否安全。
     * 点云由规划节点订阅后注入（与轨迹生成模块共享同一份点云数据）。
     */
    class CollisionChecker {
    public:
        CollisionChecker() = default;
        ~CollisionChecker() = default;

        /**
         * @brief 设置参考点云并构建 KDTree（重复调用会重建）
         *
         * @param cloud 参考点云（XYZ）
         */
        void SetCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

        /**
         * @brief 是否已有点云可检查
         */
        bool IsCloudReady() const { return cloud_ready_; }

        /**
         * @brief 点到点云的最小距离
         *
         * @param p 查询点
         * @return 最小距离；无点云时返回 -1
         */
        double MinDistanceToCloud(const Eigen::Vector3d& p) const;

        /**
         * @brief 单点位姿检查：与点云最小距离 ≥ min_clearance 才算安全
         *
         * @param pose          待检查位姿
         * @param min_clearance 安全间隙 [m]
         * @return true 安全；false 无点云或距离不足
         */
        bool CheckPose(const geometry_msgs::msg::Pose& pose, double min_clearance) const;

        /**
         * @brief 整条轨迹检查：所有点均满足安全间隙才通过
         *
         * @param trajectory    待检查轨迹
         * @param min_clearance 安全间隙 [m]
         * @return true 全部通过
         */
        bool CheckTrajectory(const Trajectory& trajectory, double min_clearance) const;

        /**
         * @brief 复位：清空点云
         */
        void Reset();

    private:
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
        pcl::KdTreeFLANN<pcl::PointXYZ> tree_;
        bool cloud_ready_ = false;
    };

}  // namespace RusSimPlanning
