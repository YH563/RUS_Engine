#include "components/kinematics.hpp"

#include <limits>

namespace RusRobotDriver {

    // ── Solver ──
    KinematicsSolver::KinematicsSolver(
        std::shared_ptr<const EAIK::Robot> ki_model,
        double flange_offset)
        : ki_model_(std::move(ki_model))
        , flange_offset_(flange_offset)
    {
        // 工具变换初始为单位阵（工具0 = 法兰坐标系）。
        // flange_offset 属于法兰坐标定义（模型末端 wrist3_link + 偏移 = 法兰），
        // 不并入工具变换；工具变换纯粹表示「工具/探头相对法兰」。
        tool_transform_.setIdentity();
    }

    // ── ForwardKinematics — 正运动学 ──
    //   T_flange = fwdkin(q) × T_flangeOffset（模型末端 + 偏移 = 法兰坐标）
    //   T_tcp    = T_flange × T_tool（工具/探头相对法兰）
    Eigen::Matrix4d KinematicsSolver::ForwardKinematics(const Eigen::VectorXd& joint_pos) const
    {
        Eigen::Matrix4d T = ki_model_->fwdkin_Eigen(joint_pos);
        // 模型末端(wrist3_link) + 法兰偏移 = 法兰位姿
        if (flange_offset_ != 0.0) {
            T(0, 3) += flange_offset_ * T(0, 2);
            T(1, 3) += flange_offset_ * T(1, 2);
            T(2, 3) += flange_offset_ * T(2, 2);
        }
        std::lock_guard<std::mutex> lock(mtx_);
        T = T * tool_transform_;
        return T;
    }

    // ── NumericalJacobian — 数值几何 Jacobian [6×n] ──
    Eigen::MatrixXd KinematicsSolver::NumericalJacobian(const Eigen::VectorXd& q) const
    {
        constexpr double kEps = 1e-6;
        int n = static_cast<int>(q.size());

        Eigen::Matrix4d T0 = ForwardKinematics(q);
        Eigen::Vector3d p0 = T0.block<3,1>(0, 3);
        Eigen::Matrix3d R0 = T0.block<3,3>(0, 0);

        Eigen::MatrixXd J(6, n);
        for (int i = 0; i < n; ++i) {
            Eigen::VectorXd q_eps = q;
            q_eps(i) += kEps;

            Eigen::Matrix4d T1 = ForwardKinematics(q_eps);
            J.block<3,1>(0, i) = (T1.block<3,1>(0, 3) - p0) / kEps;

            Eigen::Matrix3d R_rel = R0.transpose() * T1.block<3,3>(0, 0);
            Eigen::AngleAxisd aa(R_rel);
            if (aa.angle() > 1e-10)
                J.block<3,1>(3, i) = aa.axis() * aa.angle() / kEps;
            else
                J.block<3,1>(3, i) = Eigen::Vector3d::Zero();
        }
        return J;
    }

    // ── InverseKinematics — 逆运动学 ──
    //   T_flange = T_tcp × inv(T_tool)（TCP → 法兰）
    //   T_wrist3 = T_flange × inv(T_flangeOffset)（去法兰偏移 → 模型末端）
    IKS::IK_Solution KinematicsSolver::InverseKinematics(const Eigen::Matrix4d& pose) const
    {
        std::lock_guard<std::mutex> lock(mtx_);
        Eigen::Matrix4d T = pose * tool_transform_.inverse();
        if (flange_offset_ != 0.0) {
            T(0, 3) -= flange_offset_ * T(0, 2);
            T(1, 3) -= flange_offset_ * T(1, 2);
            T(2, 3) -= flange_offset_ * T(2, 2);
        }
        return ki_model_->calculate_IK(T);
    }

    // ── PickBestIK — 从 IK 多解中选取最佳解 ──
    int KinematicsSolver::PickBestIK(
        const IKS::IK_Solution& ik,
        const Eigen::VectorXd& ref,
        Eigen::VectorXd& q_out) const
    {
        int best_idx = -1;
        double best_dist = std::numeric_limits<double>::max();

        for (size_t i = 0; i < ik.Q.size(); ++i) {
            if (ik.is_LS_vec[i]) continue;

            double dist = 0.0;
            for (size_t j = 0; j < ik.Q[i].size() && j < (size_t)ref.size(); ++j) {
                double d = ik.Q[i][j] - ref(j);
                d = std::atan2(std::sin(d), std::cos(d));
                dist += d * d;
            }

            if (dist < best_dist) {
                best_dist = dist;
                best_idx = static_cast<int>(i);
            }
        }

        if (best_idx < 0) return -1;

        q_out.resize(ik.Q[best_idx].size());
        for (size_t i = 0; i < ik.Q[best_idx].size(); ++i)
            q_out(i) = ik.Q[best_idx][i];
        return best_idx;
    }

    // ── SetToolTransform — 设置工具坐标系变换（TCP 相对法兰），线程安全 ──
    void KinematicsSolver::SetToolTransform(const Eigen::Matrix4d& T)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        tool_transform_ = T;
    }

    // ── tool_transform — 当前工具变换访问器 ──
    Eigen::Matrix4d KinematicsSolver::tool_transform() const
    {
        std::lock_guard<std::mutex> lock(mtx_);
        return tool_transform_;
    }

}  // namespace RusRobotDriver
