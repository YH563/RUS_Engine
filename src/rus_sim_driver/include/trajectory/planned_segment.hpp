#pragma once

#include <memory>
#include <Eigen/Geometry>

#include "trajectory_executor.hpp"
#include "EAIK/EAIK.h"

namespace RusRobotDriver {

    /**
     * @brief 点对点规划轨迹段，继承 ITrajectorySegment
     *
     * 支持两种运动模式：
     *   - MOTION_TYPE_JOINT：关节空间 MoveJ，五次多项式插值
     *   - MOTION_TYPE_CART：笛卡尔空间直线 MoveL，slerp 插值 + IK
     * 输出完整的 ControlTarget{q_des, qd_des, qdd_des}
     */
    class PlannedSegment : public ITrajectorySegment {
    public:
        /**
         * @brief 构造规划段
         * 
         * @param cmd           运动指令（含 type, target, speed, acceleration）
         * @param start         起始机器人状态
         * @param ki_model      EAIK 运动学模型（MoveL 需要）
         * @param flange_offset 法兰偏移量
         */
        PlannedSegment(
            const MotionCommand& cmd,
            const RobotState& start,
            std::shared_ptr<const EAIK::Robot> ki_model = nullptr,
            double flange_offset = 0.0
        );

        /**
         * @brief 每帧推进一次，返回期望控制目标
         * 
         * @param sim_time 当前仿真时间
         * @param state    当前机器人状态
         * @return ControlTarget 期望关节位置、速度、加速度
         */
        ControlTarget Step(double sim_time, const RobotState& state) override;

        /**
         * @brief 判断当前段是否执行完毕
         * 
         * @return true  已到达目标
         * @return false 仍在运动中
         */
        bool IsFinished() const override;

        /**
         * @brief 获取段类型
         * 
         * @return int MOTION_TYPE_JOINT 或 MOTION_TYPE_CART
         */
        int GetType() const override;

    private:
        /**
         * @brief 根据速度/加速度比例估算轨迹总时长
         * 
         * @param start 起始状态
         * @return double 总时长 [s]
         */
        double calc_duration(const RobotState& start) const;

        /**
         * @brief 五次多项式求值
         * 
         * @param t   当前绝对时间
         * @param T   总时长
         * @param s   输出：归一化位置 [0,1]
         * @param ds  输出：s 对时间的一阶导数
         * @param dds 输出：s 对时间的二阶导数
         */
        void quintic_poly(double t, double T,
                          double& s, double& ds, double& dds) const;

        /**
         * @brief 关节空间插值：解析计算 q/qd/qdd
         */
        ControlTarget evaluate_joint(double s, double ds, double dds) const;

        /**
         * @brief 笛卡尔空间插值：插值→IK→数值 Jacobian→解析 twist
         */
        ControlTarget evaluate_cartesian(double s, double ds, double dds,
                                         const RobotState& state, double dt) const;

        /**
         * @brief 数值几何 Jacobian [6×n]：扰动 FK 差分得雅可比列
         */
        Eigen::MatrixXd compute_numerical_jacobian(const VectorXd& q) const;

        /**
         * @brief 解析笛卡尔速度 [v; ω]：从插值路径 s(t) 导数计算
         */
        Eigen::Matrix<double, 6, 1> compute_cartesian_twist(double ds) const;

        /**
         * @brief 带法兰偏移补偿的正运动学
         */
        Eigen::Matrix4d forward_kinematics(const VectorXd& joint_pos) const;

        /**
         * @brief 带法兰偏移补偿的逆运动学
         */
        IKS::IK_Solution inverse_kinematics(const Eigen::Matrix4d& pose) const;

        /**
         * @brief 从 IK 多解中选取最佳解
         *
         * 策略：
         *   1. 轨迹跟踪：后续帧以 prev_q_des_ 为参考，确保同一分支
         *   2. 角度环绕处理：各关节差值归一化到 [-π, π]
         *   3. 首帧以 state.joint_pos 为参考
         * 
         * @param ik     IK 解集
         * @param ref    参考关节角（用于选最近解）
         * @param q_out 输出：选中的关节角向量
         * @return int  选中的解索引，-1 表示无有效解
         */
        int pick_ik_solution(const IKS::IK_Solution& ik,
                             const VectorXd& ref,
                             VectorXd& q_out) const;

        // === 运动模式 ===
        uint8_t motion_type_;

        // === 时间参数 ===
        double duration_{0.0};    // 总规划时长 [s]
        double elapsed_{0.0};     // 已运行时间 [s]

        // === 关节空间轨迹参数 ===
        VectorXd start_joint_;    // 起始关节角
        VectorXd target_joint_;   // 目标关节角

        // === 笛卡尔空间轨迹参数 ===
        Eigen::Vector3d start_pos_;          // 起始位置
        Eigen::Vector3d target_pos_;         // 目标位置
        Eigen::Quaterniond start_quat_;      // 起始姿态（四元数）
        Eigen::Quaterniond target_quat_;     // 目标姿态（四元数）

        // === EAIK 运动学模型（仅 MoveL 需要） ===
        std::shared_ptr<const EAIK::Robot> ki_model_;
        double flange_offset_{0.0};

        // === 速度/加速度参数 ===
        double speed_ratio_{0.5};

        // === 上一帧结果（用于 MoveL 数值差分求 qd/qdd） ===
        mutable VectorXd prev_q_des_;
        mutable VectorXd prev_qd_des_;
        mutable double last_sim_time_{-1.0};  // 用于计算 dt
    };

}  // namespace RusRobotDriver