#pragma once

// ════════════════════════════════════════════════════════════════════
//  插值计算模块
//  ────────────────────────────────────────────────────────────────────
//  将轨迹生成模块输出的稀疏路径点稠密化：相邻路径点之间
//  位置线性插值 + 姿态球面插值（slerp），生成平滑稠密轨迹，
//  供执行 / 伺服跟踪使用（转发给驱动前调用 Interpolate()）。
// ════════════════════════════════════════════════════════════════════

#include <cstddef>
#include <optional>

#include <Eigen/Dense>
#include <geometry_msgs/msg/pose.hpp>

#include "rus_sim_utils/robot_state.hpp"
#include "rus_sim_planning/trajectory_generator.hpp"

namespace RusSimPlanning {

    /**
     * @brief 插值计算模块
     *
     * 接收轨迹生成模块的稀疏路径点，稠密化为平滑位姿轨迹供执行。
     * 位置线性插值 + 姿态 slerp；后续控制算法（轨迹修正）可在此扩展。
     */
    class TrajectoryInterpolator {
    public:
        TrajectoryInterpolator() = default;
        ~TrajectoryInterpolator() = default;

        /**
         * @brief 设置稀疏路径点（轨迹生成模块输出）
         *
         * @param waypoints 稀疏路径点
         */
        void SetWaypoints(const Trajectory& waypoints);

        /**
         * @brief 刷新机械臂状态（来自 /driver/state，供后续控制算法使用）
         *
         * @param state 机械臂状态
         */
        void SetRobotState(const RusUtils::RobotState& state);

        /**
         * @brief 插值稠密化：相邻路径点之间插入 points_per_segment 个中间点
         *
         * @param points_per_segment 每段插值点数（<1 时钳制为 1）
         * @return true 成功；false 路径点不足，无法插值
         */
        bool Interpolate(int points_per_segment = 10);

        /**
         * @brief 稠密化后的轨迹 / 原始稀疏路径点
         */
        const Trajectory& DenseTrajectory() const { return dense_; }
        const Trajectory& Waypoints() const { return waypoints_; }

        /**
         * @brief 推进输出：取当前目标并推进到下一个（伺服执行用）
         *
         * @return 当前目标位姿；轨迹为空或已推进完时返回 nullopt
         */
        std::optional<geometry_msgs::msg::Pose> NextTarget();

        /**
         * @brief 当前目标（不推进）
         *
         * @return 当前目标位姿；无轨迹时返回 nullopt
         */
        std::optional<geometry_msgs::msg::Pose> CurrentTarget() const;

        /**
         * @brief 是否已全部推进完
         */
        bool IsFinished() const;

        /**
         * @brief 稠密轨迹点数 / 当前推进索引（调试用）
         */
        std::size_t Size() const { return dense_.size(); }
        std::size_t CurrentIndex() const { return idx_; }

        /**
         * @brief 复位：清空轨迹与索引
         */
        void Reset();

        /**
         * @brief 回到轨迹起点（不清空轨迹，用于重新执行）
         */
        void Rewind() { idx_ = 0; }

        /**
         * @brief 最新机械臂状态（供后续控制算法使用）
         */
        const RusUtils::RobotState& LatestState() const { return robot_state_; }

    private:
        // 两路点间插值：位置线性 + 姿态 slerp
        static geometry_msgs::msg::Pose lerp_pose(
            const geometry_msgs::msg::Pose& a,
            const geometry_msgs::msg::Pose& b,
            double t);

        Trajectory waypoints_;       // 稀疏路径点（规划器输出）
        Trajectory dense_;           // 稠密化轨迹
        std::size_t idx_ = 0;        // 推进索引
        RusUtils::RobotState robot_state_;  // 最新机械臂状态
    };

}  // namespace RusSimPlanning
