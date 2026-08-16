#include "trajectory/planned_segment.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <Eigen/SVD>

namespace RusRobotDriver {
    // 内部常量
    namespace {
        constexpr double kBaseJointSpeed   = 1.0;   // 关节基准速度 [rad/s]
        constexpr double kBaseCartLinSpeed = 0.5;   // 笛卡尔直线基准速度 [m/s]
        constexpr double kBaseCartRotSpeed = 1.0;   // 笛卡尔旋转基准速度 [rad/s]
        constexpr double kMinDuration      = 0.05;  // 最短时长 [s]
        constexpr double kMaxDuration      = 30.0;  // 最长时长 [s]
    }

    // ---- 构造 ----
    PlannedSegment::PlannedSegment(
        const MotionCommand& cmd,
        const RobotState& start,
        std::shared_ptr<const KinematicsSolver> kinematics
    )
        : motion_type_(cmd.type)
        , kinematics_(std::move(kinematics))
        , speed_ratio_(std::clamp(cmd.speed, 0.01, 1.0))
    {
        if (motion_type_ == MOTION_TYPE_JOINT) {
            start_joint_  = start.joint_pos;
            target_joint_ = cmd.target;
            if (start_joint_.size() != target_joint_.size())
                throw std::runtime_error("PlannedSegment: joint dimension mismatch");
        }
        else if (motion_type_ == MOTION_TYPE_CART) {
            if (!kinematics_)
                throw std::runtime_error("PlannedSegment: kinematics required for MoveL");

            // 起始位姿：使用运动学求解器正解（与 IK 同源，保证一致性）
            {
                Eigen::Matrix4d T_start = kinematics_->ForwardKinematics(start.joint_pos);
                start_pos_  = T_start.block<3,1>(0, 3);
                start_quat_ = Eigen::Quaterniond(T_start.block<3,3>(0, 0));
            }

            // 目标位姿：cmd.target = [x, y, z, rx, ry, rz]
            target_pos_ = cmd.target.segment<3>(0);
            {
                double rx = cmd.target(3);
                double ry = cmd.target(4);
                double rz = cmd.target(5);
                double cx = std::cos(rx), sx_s = std::sin(rx);
                double cy = std::cos(ry), sy_s = std::sin(ry);
                double cz = std::cos(rz), sz_s = std::sin(rz);
                Eigen::Matrix3d R;
                R << cz*cy,  cz*sy_s*sx_s - sz_s*cx,  cz*sy_s*cx + sz_s*sx_s,
                     sz_s*cy,  sz_s*sy_s*sx_s + cz*cx,  sz_s*sy_s*cx - cz*sx_s,
                     -sy_s,    cy*sx_s,                cy*cx;
                target_quat_ = Eigen::Quaterniond(R);
            }
        }

        duration_ = calc_duration(start);
        elapsed_  = 0.0;
    }

    // ---- Step — 每帧推进 ----
    ControlTarget PlannedSegment::Step(double sim_time, const RobotState& state)
    {
        if (last_sim_time_ < 0.0)
            last_sim_time_ = sim_time;

        double dt = sim_time - last_sim_time_;
        last_sim_time_ = sim_time;

        // 实际用于推进的步长（传给 evaluate_cartesian 做差分基准）
        double step_dt = dt;
        if (dt > 0.0 && dt < 0.1)
            elapsed_ += dt;
        else if (dt <= 0.0)
            elapsed_ += (step_dt = 0.001);

        double t = std::min(elapsed_, duration_);

        double s, ds, dds;
        quintic_poly(t, duration_, s, ds, dds);

        if (motion_type_ == MOTION_TYPE_JOINT)
            return evaluate_joint(s, ds, dds);
        else
            return evaluate_cartesian(s, ds, dds, state, step_dt);
    }

    // ---- IsFinished / GetType ----
    bool PlannedSegment::IsFinished() const { return elapsed_ >= duration_; }
    int  PlannedSegment::GetType() const { return static_cast<int>(motion_type_); }

    // ---- calc_duration — 根据位移和速度比例估算总时长 ----
    double PlannedSegment::calc_duration(const RobotState& start) const
    {
        (void)start;

        if (motion_type_ == MOTION_TYPE_JOINT) {
            double max_disp = (target_joint_ - start_joint_).lpNorm<Eigen::Infinity>();
            return std::clamp(max_disp / (speed_ratio_ * kBaseJointSpeed), kMinDuration, kMaxDuration);
        } else {
            double lin_dist = (target_pos_ - start_pos_).norm();
            double ang_dist = start_quat_.angularDistance(target_quat_);
            double t_lin = lin_dist / (speed_ratio_ * kBaseCartLinSpeed);
            double t_ang = ang_dist / (speed_ratio_ * kBaseCartRotSpeed);
            return std::clamp(std::max(t_lin, t_ang), kMinDuration, kMaxDuration);
        }
    }

    // ---- quintic_poly — 五次多项式 ----
    // s(t)   = 10τ³ - 15τ⁴ + 6τ⁵,  τ = t/T
    // 边界条件：s(0)=s'(0)=s''(0)=0, s(T)=1, s'(T)=s''(T)=0
    void PlannedSegment::quintic_poly(double t, double T,
                                      double& s, double& ds, double& dds) const
    {
        if (T <= 0.0 || t >= T) { s = 1.0; ds = 0.0; dds = 0.0; return; }

        double tau = t / T, tau2 = tau * tau, tau3 = tau2 * tau;
        s   = 10.0 * tau3 - 15.0 * tau3 * tau + 6.0 * tau3 * tau2;
        ds  = (30.0 * tau2 - 60.0 * tau3 + 30.0 * tau3 * tau) / T;
        dds = (60.0 * tau - 180.0 * tau2 + 120.0 * tau3) / (T * T);
    }

    // ---- evaluate_joint — 关节空间插值（解析求导） ----
    ControlTarget PlannedSegment::evaluate_joint(double s, double ds, double dds) const
    {
        ControlTarget target;
        VectorXd delta = target_joint_ - start_joint_;
        target.q_des   = start_joint_ + delta * s;
        target.qd_des  = delta * ds;
        target.qdd_des = delta * dds;
        return target;
    }

    // ---- evaluate_cartesian — 笛卡尔插值 → IK → 解析速度（Jacobian × twist） ----
    ControlTarget PlannedSegment::evaluate_cartesian(double s, double ds, double dds,
                                                      const RobotState& state, double dt) const
    {
        ControlTarget target;

        // ── 轨迹结束锁定：s ≥ 1 时复用上一帧的 IK 解，避免末尾 IK 抖动 ──
        if (s >= 1.0 && prev_q_des_.size() > 0) {
            target.q_des   = prev_q_des_;
            target.qd_des  = VectorXd::Zero(prev_q_des_.size());
            target.qdd_des = VectorXd::Zero(prev_q_des_.size());
            prev_qd_des_   = target.qd_des;
            return target;
        }

        // 插值位姿
        Eigen::Vector3d pos = start_pos_ + (target_pos_ - start_pos_) * s;
        Eigen::Quaterniond quat = start_quat_.slerp(s, target_quat_);
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3,3>(0, 0) = quat.toRotationMatrix();
        T.block<3,1>(0, 3) = pos;

        // IK 求解（KinematicsSolver 内部处理法兰偏移补偿）
        IKS::IK_Solution ik = kinematics_->InverseKinematics(T);
        if (ik.Q.empty()) {
            std::fprintf(stderr,
                "[PlannedSegment] WARN: IK 无解（目标 pos %.3f,%.3f,%.3f），保持当前位置\n",
                T(0,3), T(1,3), T(2,3));
            target.q_des   = state.joint_pos;
            target.qd_des  = VectorXd::Zero(state.joint_pos.size());
            target.qdd_des = VectorXd::Zero(state.joint_pos.size());
            return target;
        }

        // 选解：首帧以 state.joint_pos 为参考，后续跟踪 prev_q_des_ 锁定分支
        VectorXd q_curr;
        const VectorXd& ref = (prev_q_des_.size() > 0) ? prev_q_des_ : state.joint_pos;
        int best_idx = kinematics_->PickBestIK(ik, ref, q_curr);
        if (best_idx < 0) {
            target.q_des   = state.joint_pos;
            target.qd_des  = VectorXd::Zero(state.joint_pos.size());
            target.qdd_des = VectorXd::Zero(state.joint_pos.size());
            return target;
        }
        target.q_des = q_curr;

        // ---- 解析速度：qd = J⁻¹ · twist（动态阻尼 DLS） ----
        Eigen::MatrixXd J = kinematics_->NumericalJacobian(q_curr);
        Eigen::Matrix<double, 6, 1> twist = compute_cartesian_twist(ds);
        target.qd_des = damped_least_squares(J, twist, 0.001, 0.2, 0.02);

        // ---- 加速度：后向差分 qd ----
        if (dt <= 0.0) dt = 0.001;
        if (prev_qd_des_.size() == q_curr.size()) {
            target.qdd_des = (target.qd_des - prev_qd_des_) / dt;
        } else {
            target.qdd_des = VectorXd::Zero(q_curr.size());
            // 第一帧同时用解析加速度近似
            Eigen::Matrix<double, 6, 1> twist_dot = compute_cartesian_twist(dds);
            target.qdd_des = damped_least_squares(J, twist_dot, 0.001, 0.2, 0.02);
        }
        prev_qd_des_ = target.qd_des;
        prev_q_des_  = q_curr;
        return target;
    }

    // ---- compute_cartesian_twist — 解析笛卡尔速度 [v; ω] ----
    //  线速度: v = (p₁ - p₀) · s'(t)
    //  角速度: q_rel = q₀̄ · q₁ → axis, angle
    //          ω_world = R(q₀) · axis · angle · s'(t)
    Eigen::Matrix<double, 6, 1> PlannedSegment::compute_cartesian_twist(double ds) const
    {
        Eigen::Matrix<double, 6, 1> twist;

        // 线速度
        twist.segment<3>(0) = (target_pos_ - start_pos_) * ds;

        // 角速度：从起始到目标的相对旋转
        Eigen::Quaterniond q_rel = start_quat_.conjugate() * target_quat_;
        if (q_rel.w() >= 1.0 - 1e-10) {
            twist.segment<3>(3) = Eigen::Vector3d::Zero();
        } else {
            double angle = 2.0 * std::acos(std::clamp(q_rel.w(), -1.0, 1.0));
            Eigen::Vector3d axis(q_rel.x(), q_rel.y(), q_rel.z());
            axis.normalize();
            // ω_world = R(q₀) · axis · θ · ds
            twist.segment<3>(3) = start_quat_.toRotationMatrix() * axis * angle * ds;
        }
        return twist;
    }

}  // namespace RusRobotDriver