#pragma once

#include <memory>
#include <Eigen/Geometry>

#include "trajectory_executor.hpp"
#include "EAIK/EAIK.h"

namespace RusRobotDriver {

    /**
     * @brief 点动轨迹段，继承 ITrajectorySegment
     *
     * 支持三种点动模式：
     *   - MOTION_TYPE_JOG_0：关节空间点动，沿指定关节方向匀速运动
     *   - MOTION_TYPE_JOG_1：基坐标系点动，沿基坐标轴平移/旋转
     *   - MOTION_TYPE_JOG_2：工具坐标系点动，沿工具坐标轴平移/旋转
     *
     * 每帧按恒定速度积分位置，输出 q/qd（qdd = 0）。
     * 当 jog_max_dis > 0 时，到达累积位移上限后自动结束。
     */
    class JogSegment : public ITrajectorySegment {
    public:
        /**
         * @brief 构造点动段
         *
         * @param cmd           运动指令（含 JOG 参数）
         * @param state         当前机器人状态
         * @param ki_model      EAIK 运动学模型（JOG_1/2 需要）
         * @param flange_offset 法兰偏移量
         */
        JogSegment(const MotionCommand& cmd, const RobotState& state,
                   std::shared_ptr<const EAIK::Robot> ki_model = nullptr,
                   double flange_offset = 0.0);

        /**
         * @brief 更新点动参数（轴号/方向/速度等）
         *
         * @param cmd   新的运动指令
         * @param state 当前机器人状态
         */
        void UpdateTarget(const MotionCommand& cmd, const RobotState& state) override;

        /**
         * @brief 执行一帧点动，返回期望控制目标
         */
        ControlTarget Step(double sim_time, const RobotState& state) override;

        /**
         * @brief 检查是否到达最大位移上限
         *
         * @return true  jog_max_dis > 0 且累积位移已达上限
         * @return false 继续运动
         */
        bool IsFinished() const override;

        /**
         * @brief 获取段类型：MOTION_TYPE_JOG_0/1/2
         */
        int GetType() const override;

    private:
        uint8_t motion_type_;       // MOTION_TYPE_JOG_0/1/2
        uint8_t jog_axis_;          // 轴号 1~6
        uint8_t jog_dir_;           // 0-负方向，1-正方向
        double speed_ratio_;        // 速度比例 [0~1]
        double max_dis_;            // 最大位移，0=无限制
        double accumulated_dis_ = 0.0;  // 已累积位移
        double last_sim_time_{-1.0};

        // 输出状态
        VectorXd q_des_;
        VectorXd qd_des_;

        // 运动学
        std::shared_ptr<const EAIK::Robot> ki_model_;
        double flange_offset_{0.0};

        static constexpr double kBaseSpeed      = 0.5;   // [rad/s] 关节空间基准速度
        static constexpr double kCartTransSpeed = 0.05;  // [m/s]   笛卡尔平移基准速度
        static constexpr double kCartRotSpeed   = 0.3;   // [rad/s] 笛卡尔旋转基准速度

        /** @brief 关节空间点动：直接积分目标关节角 */
        void step_joint_jog(double dt);

        /** @brief 笛卡尔空间点动：FK→位姿偏移→IK */
        void step_cartesian_jog(double dt, const RobotState& state);

        /** @brief 正运动学（含法兰偏移补偿） */
        Eigen::Matrix4d forward_kinematics(const VectorXd& joint_pos) const;

        /** @brief 数值几何 Jacobian [6×n] */
        Eigen::MatrixXd compute_numerical_jacobian(const VectorXd& q) const;
    };

}  // namespace RusRobotDriver
