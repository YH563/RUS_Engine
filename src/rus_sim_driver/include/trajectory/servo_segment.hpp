#pragma once

#include <memory>
#include <Eigen/Geometry>

#include "trajectory_executor.hpp"
#include "EAIK/EAIK.h"

namespace RusRobotDriver {

    /**
     * @brief 伺服轨迹段，继承 ITrajectorySegment
     *
     * 支持两种模式：
     *   - MOTION_TYPE_SERVOJ：关节空间伺服，cmd.target 为关节角 [rad]
     *   - MOTION_TYPE_SERVOC：笛卡尔空间伺服，cmd.target = [x,y,z,rx,ry,rz]
     *     自动做 IK 转换到关节角。
     *
     * 连续帧间通过二阶低通滤波器对关节角进行平滑，输出完整的 q/qd/qdd。
     * 同时对每帧目标做位移钳位，防止目标跳变过大导致危险。
     *
     * 滤波器模型（连续域）：
     *   q̈ = ωₙ² · (q_raw - q) - 2ζωₙ · q̇
     * 离散化使用半隐式欧拉法。
     */
    class ServoSegment : public ITrajectorySegment {
    public:
        /**
         * @brief 构造伺服段，初始化滤波器状态
         *
         * @param cmd           首帧运动指令（MOTION_TYPE_SERVOJ 或 SERVOC）
         * @param state         当前机器人状态
         * @param ki_model      EAIK 运动学模型（ServoCart 需要）
         * @param flange_offset 法兰偏移量
         */
        ServoSegment(const MotionCommand& cmd,
                     const RobotState& state,
                     std::shared_ptr<const EAIK::Robot> ki_model,
                     double flange_offset = 0.0);

        /**
         * @brief 每帧用最新目标更新滤波器
         *
         * 在 Step() 之前由调度器调用，传入当前帧的伺服指令。
         *
         * @param cmd   当前帧伺服指令
         * @param state 当前机器人状态（IK 选解参考）
         */
        void UpdateTarget(const MotionCommand& cmd, const RobotState& state) override;

        /**
         * @brief 执行一帧滤波，返回平滑后的控制目标
         */
        ControlTarget Step(double sim_time, const RobotState& state) override;

        /**
         * @brief 伺服段持续存在，IsFinished 始终返回 false
         */
        bool IsFinished() const override;

        /**
         * @brief 获取段类型：MOTION_TYPE_SERVOJ 或 MOTION_TYPE_SERVOC
         */
        int GetType() const override;

    private:
        uint8_t motion_type_;  // MOTION_TYPE_SERVOJ 或 MOTION_TYPE_SERVOC

        // 原始目标（滤波器输入）
        VectorXd q_raw_;

        // 二阶滤波器状态
        VectorXd q_filt_;   // 滤波后关节角
        VectorXd qd_filt_;  // 滤波估计速度
        double last_sim_time_{-1.0};  // 上一帧时间

        // 滤波器参数
        static constexpr double kFilterWn  = 30.0;   // 自然频率 [rad/s] ≈ 5Hz
        static constexpr double kFilterZeta = 1.0;   // 阻尼比，1=临界阻尼

        // 每帧最大关节位移 [rad]，防止目标跳变过大
        static constexpr double kMaxStep = 0.05;

        // 运动学
        std::shared_ptr<const EAIK::Robot> ki_model_;
        double flange_offset_{0.0};

        /**
         * @brief 执行二阶低通滤波单步递推
         */
        void apply_filter(double dt);

        /**
         * @brief 限制目标相对于参考位置的每帧最大变化量
         *
         * @param target    原始目标关节角
         * @param reference 参考位置（当前滤波位置）
         * @return 钳位后的关节角
         */
        VectorXd clamp_displacement(const VectorXd& target, const VectorXd& reference) const;

        /**
         * @brief 笛卡尔位姿 → 关节角（IK，仅 ServoCart 使用）
         */
        VectorXd solve_ik(const MotionCommand& cmd,
                          const RobotState& state) const;

        /**
         * @brief 带法兰偏移补偿的逆运动学
         */
        IKS::IK_Solution inverse_kinematics(const Eigen::Matrix4d& pose) const;

        /**
         * @brief 从 IK 多解中选距离参考关节角最近的解
         */
        int pick_best_ik(const IKS::IK_Solution& ik,
                         const VectorXd& ref,
                         VectorXd& q_out) const;
    };

}  // namespace RusRobotDriver