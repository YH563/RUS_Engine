#pragma once

#include <deque>
#include <memory>

#include "components/types.hpp"
#include "EAIK/EAIK.h"

namespace RusRobotDriver {

    /**
     * @brief 轨迹段抽象基类
     *
     * 每个 ITrajectorySegment 代表一段完整的运动
     * （一次 MoveJ / MoveL 或一帧伺服指令）。
     * 调度器每帧调用 Step() 获取期望状态，
     * 通过 IsFinished() 判断是否自动切换到下一条指令。
     */
    class ITrajectorySegment {
    public:
        virtual ~ITrajectorySegment() = default;

        /**
         * @brief 传入当前仿真时间和机器人状态，输出下一帧控制目标
         * 
         * @param sim_time 当前仿真时间
         * @param state    当前机器人状态
         * @return ControlTarget 期望关节位置、速度、加速度
         */
        virtual ControlTarget Step(double sim_time, const RobotState& state) = 0;

        /**
         * @brief 判断当前段是否执行完毕
         * 
         * @return true  已到达目标，可切换下一条指令
         * @return false 仍在运动中
         */
        virtual bool IsFinished() const = 0;

        /**
         * @brief 获取段类型，用于调试日志
         * 
         * @return int MOTION_TYPE_JOINT / MOTION_TYPE_CART / MOTION_TYPE_SERVO
         */
        virtual int GetType() const = 0;

        /**
         * @brief 更新段目标（伺服段每帧调用，规划段无需实现）
         * 
         * @param cmd   当前帧指令
         * @param state 当前状态
         */
        virtual void UpdateTarget(const MotionCommand& cmd, const RobotState& state) {}
    };

    // 继承自 ITrajectorySegment 的具体段类型（前置声明）
    class PlannedSegment;   // MoveJ / MoveL —— 完整点对点轨迹，含速度/加速度规划
    class ServoSegment;     // ServoJ / ServoCart —— 单帧增量伺服
    class JogSegment;       // Jog —— 恒定速度点动

    // ── 速度求解工具 ──

    /**
     * @brief 阻尼最小二乘求解 J * qd = twist
     *
     * 根据最小奇异值自动调节阻尼：
     *   min_sv > threshold → 基础阻尼
     *   min_sv < threshold → 阻尼线性增大，防止奇异速度爆炸
     *
     * @param J          [6×n] 几何 Jacobian
     * @param twist      [6×1] 笛卡尔 twist
     * @param damp_base  基础阻尼（远离奇异点时）
     * @param damp_max   最大阻尼
     * @param sv_thresh  奇异阈值（最小奇异值低于此值时开始增加阻尼）
     * @return VectorXd  关节速度 qd
     */
    inline VectorXd damped_least_squares(
        const Eigen::MatrixXd& J,
        const Eigen::Matrix<double, 6, 1>& twist,
        double damp_base = 0.05,
        double damp_max  = 0.5,
        double sv_thresh = 0.02)
    {
        // SVD 获取最小奇异值
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
        double min_sv = svd.singularValues()(svd.singularValues().size() - 1);

        // 动态阻尼：奇异值越小阻尼越大
        double lambda = damp_base;
        if (min_sv < sv_thresh) {
            double slope = (damp_max - damp_base) / sv_thresh;
            lambda = damp_base + slope * (sv_thresh - min_sv);
        }
        lambda = std::min(lambda, damp_max);

        Eigen::MatrixXd JtJ = J.transpose() * J;
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(JtJ.rows(), JtJ.cols());
        return (JtJ + lambda * lambda * I).inverse() * J.transpose() * twist;
    }


    /**
     * @brief 轨迹调度器
     *
     * 职责：
     *   1. 管理指令队列（规划指令排尾，伺服指令插头）
     *   2. 将 MotionCommand 转换为对应的 ITrajectorySegment
     *   3. 每帧推进活跃段，输出 ControlTarget{q_des, qd_des, qdd_des}
     *   4. 支持暂停/恢复/停止等生命周期控制
     */
    class TrajectoryExecutor {
    public:
        /**
         * @brief 构造轨迹调度器
         * 
         * @param ki_model      EAIK 运动学模型（MoveL 需要）
         * @param flange_offset 法兰偏移量（可选，默认 0）
         */
        TrajectoryExecutor(std::shared_ptr<const EAIK::Robot> ki_model = nullptr,
                           double flange_offset = 0.0);

        // === 指令输入 ===

        /**
         * @brief 规划指令入队尾（MoveJ / MoveL），当前段执行完后自动消费
         * 
         * @param cmd 运动指令
         */
        void PushBack(const MotionCommand& cmd);

        /**
         * @brief 伺服指令插队首（ServoJ / ServoCart），每帧最多消费一条
         * 
         * @param cmd 运动指令
         */
        void PushFront(const MotionCommand& cmd);

        // === 生命周期控制 ===

        /**
         * @brief 启动调度器（从 STOPPED → RUNNING）
         */
        void Start();

        /**
         * @brief 暂停当前运动，保持当前位置；调用 Resume() 可继续未完成的段
         */
        void Pause();

        /**
         * @brief 恢复被暂停的运动
         */
        void Resume();

        /**
         * @brief 停止所有运动，清空指令队列和活跃段，回到 STOPPED 状态
         */
        void Stop();

        /**
         * @brief 清空指令队列但让当前段继续执行完
         */
        void ClearQueue();

        // === 点动控制 ===

        /**
         * @brief 减速停止点动：速度逐渐降为 0 后自动退出点动段
         */
        void StopJOGDecel();

        /**
         * @brief 直接停止点动：立即销毁当前点动段，停在原地
         */
        void StopJOGImmediate();

        /**
         * @brief 查询是否有活跃段正在执行
         * 
         * @return true  有活跃段且未结束
         * @return false 空闲或已停止
         */
        bool IsActive() const;

        // === 主循环接口 ===

        /**
         * @brief 每帧调用一次，推进轨迹并返回期望控制目标
         * 
         * @param sim_time 当前仿真时间
         * @param state    当前机器人状态
         * @return ControlTarget 包含期望位置、速度、加速度
         */
        ControlTarget Step(double sim_time, const RobotState& state);

    private:
        /**
         * @brief 从队首取一条 MotionCommand，构造对应的 ITrajectorySegment 并设为活跃段
         * 
         * @param state 当前机器人状态
         */
        void load_next_segment(const RobotState& state);

        /**
         * @brief 消费队首的伺服指令（MOTION_TYPE_SERVOJ/SERVOC），结果写入 hold_target_
         * 
         * @param state 当前机器人状态
         */
        void process_servo_commands(const RobotState& state);

        /**
         * @brief 消费队首的点动指令（MOTION_TYPE_JOG_*），更新或创建 JogSegment
         * 
         * @param state 当前机器人状态
         */
        void process_jog_commands(const RobotState& state);

        /** @brief 从队列中清空所有点动指令 */
        void clear_jog_commands();

        // === 运动学模型 ===
        std::shared_ptr<const EAIK::Robot> ki_model_;
        double flange_offset_{0.0};

        // === 调度器状态 ===
        enum class ExecState { STOPPED, RUNNING, PAUSED };
        ExecState exec_state_{ExecState::STOPPED};

        // === 数据结构 ===
        std::deque<MotionCommand> command_queue_;          // 待消费指令队列
        std::unique_ptr<ITrajectorySegment> active_segment_;  // 当前活跃轨迹段
        ControlTarget hold_target_;                        // PAUSED / STOPPED 时的保持目标

        // === 点动减速状态 ===
        bool jog_decel_active_ = false;                    // 减速进行中
        double jog_decel_speed_ = 0.0;                     // 当前减速阶段的速度比例
        MotionCommand last_jog_cmd_;                       // 最后一条点动指令（用于减速恢复参数）
        double last_step_time_ = -1.0;                     // 上一帧仿真时间，用于 dt
        static constexpr double kDecelRate = 2.0;          // 减速速率 [ratio/s]
    };

}  // namespace RusRobotDriver