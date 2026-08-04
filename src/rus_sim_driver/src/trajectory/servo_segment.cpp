#include "trajectory/servo_segment.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace RusRobotDriver {

    // ---- 构造 ----
    ServoSegment::ServoSegment(const MotionCommand& cmd,
                               const RobotState& state,
                               std::shared_ptr<const KinematicsSolver> kinematics)
        : motion_type_(cmd.type)
        , kinematics_(std::move(kinematics))
    {
        // 根据模式获取首帧目标关节角
        VectorXd q_target;
        if (motion_type_ == MOTION_TYPE_SERVOJ) {
            q_target = cmd.target;
        } else {  // MOTION_TYPE_SERVOC
            q_target = solve_ik(cmd, state);
        }

        // 安全钳位：首帧目标与当前位置相差不能太大
        q_raw_ = clamp_displacement(q_target, state.joint_pos);

        // 初始化滤波器状态
        q_filt_  = q_raw_;
        qd_filt_ = VectorXd::Zero(q_raw_.size());
    }

    // ---- UpdateTarget — 每帧更新目标 ----
    void ServoSegment::UpdateTarget(const MotionCommand& cmd, const RobotState& state)
    {
        VectorXd q_target;
        if (cmd.type == MOTION_TYPE_SERVOJ) {
            q_target = cmd.target;
        } else {  // MOTION_TYPE_SERVOC
            q_target = solve_ik(cmd, state);
        }

        // 安全钳位：相对于当前滤波位置限制每帧变化量
        q_raw_ = clamp_displacement(q_target, q_filt_);
    }

    // ---- Step — 执行一帧滤波 ----
    ControlTarget ServoSegment::Step(double sim_time, const RobotState& /*state*/)
    {
        // 计算 dt
        if (last_sim_time_ < 0.0)
            last_sim_time_ = sim_time;
        double dt = sim_time - last_sim_time_;
        last_sim_time_ = sim_time;

        if (dt <= 0.0 || dt > 0.1) dt = 0.001;

        // 执行二阶滤波
        apply_filter(dt);

        ControlTarget target;
        target.q_des   = q_filt_;
        target.qd_des  = qd_filt_;

        // 加速度由滤波器状态直接计算
        // q̈ = ωₙ² · (q_raw - q_filt) - 2ζωₙ · qd_filt
        target.qdd_des = kFilterWn * kFilterWn * (q_raw_ - q_filt_)
                       - 2.0 * kFilterZeta * kFilterWn * qd_filt_;

        return target;
    }

    // ---- IsFinished — 伺服段持续存在 ----
    bool ServoSegment::IsFinished() const { return false; }
    int  ServoSegment::GetType() const { return motion_type_; }

    // ---- apply_filter — 二阶低通滤波单步递推 ----
    //  连续模型: q̈ = ωₙ²·(q_raw - q) - 2ζωₙ·q̇
    //  半隐式欧拉离散化:
    //    qd += dt · q̈
    //    q  += dt · qd
    void ServoSegment::apply_filter(double dt)
    {
        for (int i = 0; i < q_filt_.size(); ++i) {
            double acc = kFilterWn * kFilterWn * (q_raw_(i) - q_filt_(i))
                       - 2.0 * kFilterZeta * kFilterWn * qd_filt_(i);
            qd_filt_(i) += dt * acc;
            q_filt_(i)  += dt * qd_filt_(i);
        }
    }

    // ---- clamp_displacement — 每帧位移安全钳位 ----
    VectorXd ServoSegment::clamp_displacement(const VectorXd& target,
                                              const VectorXd& reference) const
    {
        VectorXd delta = target - reference;
        for (int i = 0; i < delta.size(); ++i) {
            delta(i) = std::clamp(delta(i), -kMaxStep, kMaxStep);
        }
        return reference + delta;
    }

    // ---- solve_ik — 笛卡尔位姿 → 关节角 ----
    VectorXd ServoSegment::solve_ik(const MotionCommand& cmd,
                                    const RobotState& state) const
    {
        // cmd.target = [x, y, z, rx, ry, rz]
        double rx = cmd.target(3);
        double ry = cmd.target(4);
        double rz = cmd.target(5);

        double cx = std::cos(rx), sx = std::sin(rx);
        double cy = std::cos(ry), sy = std::sin(ry);
        double cz = std::cos(rz), sz = std::sin(rz);

        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T(0, 3) = cmd.target(0);
        T(1, 3) = cmd.target(1);
        T(2, 3) = cmd.target(2);
        T.block<3,3>(0, 0) << cz*cy,  cz*sy*sx - sz*cx,  cz*sy*cx + sz*sx,
                              sz*cy,  sz*sy*sx + cz*cx,  sz*sy*cx - cz*sx,
                              -sy,    cy*sx,             cy*cx;

        IKS::IK_Solution ik = kinematics_->InverseKinematics(T);
        if (ik.Q.empty())
            return state.joint_pos;  // IK 无解，保持当前位置

        VectorXd q_out;
        int ret = kinematics_->PickBestIK(ik, state.joint_pos, q_out);
        if (ret < 0) {
            // 所有解都是最小二乘近似（目标位姿不可达），保持当前位置
            return state.joint_pos;
        }
        return q_out;
    }

}  // namespace RusRobotDriver