#include "trajectory/jog_segment.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace RusRobotDriver {

    // ---- 构造 ----
    JogSegment::JogSegment(const MotionCommand& cmd, const RobotState& state,
                           std::shared_ptr<const EAIK::Robot> ki_model,
                           double flange_offset)
        : motion_type_(cmd.type)
        , jog_axis_(cmd.jog_axis)
        , jog_dir_(cmd.jog_dir)
        , speed_ratio_(std::clamp(cmd.speed, 0.0, 1.0))
        , max_dis_(cmd.jog_max_dis)
        , ki_model_(std::move(ki_model))
        , flange_offset_(flange_offset)
    {
        // 初始位置等于当前关节角
        q_des_  = state.joint_pos;
        qd_des_ = VectorXd::Zero(q_des_.size());
    }

    // ---- UpdateTarget — 更新点动参数 ----
    void JogSegment::UpdateTarget(const MotionCommand& cmd, const RobotState& /*state*/)
    {
        jog_axis_   = cmd.jog_axis;
        jog_dir_    = cmd.jog_dir;
        speed_ratio_ = std::clamp(cmd.speed, 0.0, 1.0);
        max_dis_    = cmd.jog_max_dis;
        // 注意：不重置 accumulated_dis_，保持连续性
    }

    // ---- Step — 执行一帧点动 ----
    ControlTarget JogSegment::Step(double sim_time, const RobotState& state)
    {
        if (last_sim_time_ < 0.0)
            last_sim_time_ = sim_time;
        double dt = sim_time - last_sim_time_;
        last_sim_time_ = sim_time;

        if (dt <= 0.0 || dt > 0.1) dt = 0.001;

        if (motion_type_ == MOTION_TYPE_JOG_0) {
            step_joint_jog(dt);
        } else {
            step_cartesian_jog(dt, state);
        }

        ControlTarget target;
        target.q_des   = q_des_;
        target.qd_des  = qd_des_;
        target.qdd_des = VectorXd::Zero(q_des_.size());
        return target;
    }

    // ---- IsFinished — 累积位移达上限时结束 ----
    bool JogSegment::IsFinished() const
    {
        return max_dis_ > 0.0 && accumulated_dis_ >= max_dis_;
    }

    int JogSegment::GetType() const { return motion_type_; }

    // ---- step_joint_jog — 关节空间点动 ----
    void JogSegment::step_joint_jog(double dt)
    {
        double step = speed_ratio_ * kBaseSpeed * dt;
        if (jog_dir_ == 0) step = -step;

        q_des_(jog_axis_ - 1) += step;
        accumulated_dis_ += std::abs(step);

        // 速度
        qd_des_.setZero();
        qd_des_(jog_axis_ - 1) = step / dt;
    }

    // ---- step_cartesian_jog — 笛卡尔空间点动 ----
    void JogSegment::step_cartesian_jog(double dt, const RobotState& /*state*/)
    {
        // 1. FK 得到 TCP 位姿（与 PlannedSegment 约定一致）
        Eigen::Matrix4d T = forward_kinematics(q_des_);

        double base_speed = (jog_axis_ <= 3) ? kCartTransSpeed : kCartRotSpeed;
        double step_mag = speed_ratio_ * base_speed * dt;
        if (jog_dir_ == 0) step_mag = -step_mag;

        int axis_idx = jog_axis_ - 1;  // 0=X,1=Y,2=Z | 3=Rx,4=Ry,5=Rz

        if (jog_axis_ <= 3) {
            // ---- 平移：在 TCP 位姿上加增量 ----
            Eigen::Vector3d dir;
            if (motion_type_ == MOTION_TYPE_JOG_1) {
                dir = Eigen::Vector3d::Unit(axis_idx);
            } else {
                // 旋转矩阵列：col(0)=X, col(1)=-Z, col(2)=Y
                static const int j2_col[] = {0, 2, 1};  // X→col0, Y→col2, Z→col1
                dir = T.block<3, 3>(0, 0).col(j2_col[axis_idx]);
                if (axis_idx == 2) dir = -dir;  // col(1) = -Z → 取反得 +Z
            }
            T.block<3, 1>(0, 3) += step_mag * dir;
        } else {
            // ---- 旋转：在 TCP 姿态上加增量 ----
            Eigen::Vector3d axis;
            int rot_idx = axis_idx - 3;  // 0=Rx,1=Ry,2=Rz
            if (motion_type_ == MOTION_TYPE_JOG_1) {
                axis = Eigen::Vector3d::Unit(rot_idx);
            } else {
                static const int j2_col[] = {0, 2, 1};  // Rx→col0, Ry→col2, Rz→col1
                axis = T.block<3, 3>(0, 0).col(j2_col[rot_idx]);
                if (rot_idx == 2) axis = -axis;  // col(1) = -Z → 取反得 +Z 旋转轴
            }
            Eigen::AngleAxisd rot(step_mag, axis);
            T.block<3, 3>(0, 0) = rot.toRotationMatrix() * T.block<3, 3>(0, 0);
        }
        accumulated_dis_ += std::abs(step_mag);

        // 2. IK：TCP 位姿 → 法兰位姿（与 PlannedSegment::inverse_kinematics 一致）
        if (flange_offset_ != 0.0) {
            T(0, 3) -= flange_offset_ * T(0, 2);
            T(1, 3) -= flange_offset_ * T(1, 2);
            T(2, 3) -= flange_offset_ * T(2, 2);
        }
        IKS::IK_Solution ik = ki_model_->calculate_IK(T);
        if (!ik.Q.empty()) {
            double best_dist = std::numeric_limits<double>::max();
            int best_idx = -1;
            for (size_t i = 0; i < ik.Q.size(); ++i) {
                if (ik.is_LS_vec[i]) continue;
                double dist = 0.0;
                for (size_t j = 0; j < ik.Q[i].size() && j < (size_t)q_des_.size(); ++j) {
                    double d = ik.Q[i][j] - q_des_(j);
                    d = std::atan2(std::sin(d), std::cos(d));
                    dist += d * d;
                }
                if (dist < best_dist) {
                    best_dist = dist;
                    best_idx = static_cast<int>(i);
                }
            }
            if (best_idx >= 0) {
                for (size_t i = 0; i < ik.Q[best_idx].size(); ++i)
                    q_des_(i) = ik.Q[best_idx][i];
            }
        }
        // 3. 通过雅可比映射笛卡尔速度 → 关节速度
        {
            Eigen::Matrix<double, 6, 1> twist;
            twist.setZero();
            double vel = speed_ratio_ * base_speed;
            if (jog_dir_ == 0) vel = -vel;

            if (jog_axis_ <= 3) {
                Eigen::Vector3d dir;
                if (motion_type_ == MOTION_TYPE_JOG_1)
                    dir = Eigen::Vector3d::Unit(axis_idx);
                else {
                    static const int j2_col[] = {0, 2, 1};
                    dir = T.block<3, 3>(0, 0).col(j2_col[axis_idx]);
                    if (axis_idx == 2) dir = -dir;
                }
                twist.segment<3>(0) = vel * dir;
            } else {
                Eigen::Vector3d axis;
                int rot_idx = axis_idx - 3;
                if (motion_type_ == MOTION_TYPE_JOG_1)
                    axis = Eigen::Vector3d::Unit(rot_idx);
                else {
                    static const int j2_col[] = {0, 2, 1};
                    axis = T.block<3, 3>(0, 0).col(j2_col[rot_idx]);
                    if (rot_idx == 2) axis = -axis;
                }
                twist.segment<3>(3) = vel * axis;
            }

            // TCP 雅可比 + 动态阻尼最小二乘（DLS）
            Eigen::MatrixXd J = compute_numerical_jacobian(q_des_);
            qd_des_ = damped_least_squares(J, twist, 0.05, 0.5, 0.02);
        }
    }

    // ---- forward_kinematics — 正运动学（含法兰偏移补偿） ----
    //  返回 TCP 位姿：p_tcp = p_flange + offset * R(:,2)（与 PlannedSegment 一致）
    Eigen::Matrix4d JogSegment::forward_kinematics(const VectorXd& joint_pos) const
    {
        Eigen::Matrix4d T = ki_model_->fwdkin_Eigen(joint_pos);

        if (flange_offset_ != 0.0) {
            T(0, 3) += flange_offset_ * T(0, 2);
            T(1, 3) += flange_offset_ * T(1, 2);
            T(2, 3) += flange_offset_ * T(2, 2);
        }
        return T;
    }

    // ---- compute_numerical_jacobian — 数值几何 Jacobian [6×n] ----
    //  返回 TCP 雅可比（与 forward_kinematics 一致）
    Eigen::MatrixXd JogSegment::compute_numerical_jacobian(const VectorXd& q) const
    {
        constexpr double kEps = 1e-6;
        int n = static_cast<int>(q.size());

        Eigen::Matrix4d T0 = forward_kinematics(q);
        Eigen::Vector3d p0 = T0.block<3, 1>(0, 3);
        Eigen::Matrix3d R0 = T0.block<3, 3>(0, 0);

        Eigen::MatrixXd J(6, n);
        for (int i = 0; i < n; ++i) {
            VectorXd q_eps = q;
            q_eps(i) += kEps;

            Eigen::Matrix4d T1 = forward_kinematics(q_eps);
            J.block<3, 1>(0, i) = (T1.block<3, 1>(0, 3) - p0) / kEps;

            Eigen::Matrix3d R_rel = R0.transpose() * T1.block<3, 3>(0, 0);
            Eigen::AngleAxisd aa(R_rel);
            if (aa.angle() > 1e-10)
                J.block<3, 1>(3, i) = aa.axis() * aa.angle() / kEps;
            else
                J.block<3, 1>(3, i) = Eigen::Vector3d::Zero();
        }
        return J;
    }

}  // namespace RusRobotDriver
