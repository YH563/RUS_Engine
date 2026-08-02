#include "controller/ctc_controller.hpp"

namespace RusRobotDriver {

    CtcController::CtcController(DynamicsFunc dynamics,
                                 double kp, double kd, double kp_ff)
        : dynamics_(std::move(dynamics))
        , kp_(kp), kd_(kd), kp_ff_(kp_ff)
    {}

    VectorXd CtcController::ComputeTorque(const ControlTarget& target,
                                          const RobotState& state)
    {
        int n = static_cast<int>(state.joint_pos.size());

        // 获取动力学
        Eigen::MatrixXd M(n, n);
        VectorXd bias(n);
        dynamics_(state.joint_pos, state.joint_vel, M, bias);

        // 期望加速度
        VectorXd a_des(n);
        if (target.qdd_des.size() == n)
            a_des = kp_ff_ * target.qdd_des;
        else
            a_des = VectorXd::Zero(n);

        if (target.q_des.size() == n)
            a_des += kp_ * (target.q_des - state.joint_pos);
        if (target.qd_des.size() == n)
            a_des += kd_ * (target.qd_des - state.joint_vel);

        return M * a_des + bias;
    }

    void CtcController::SetGain(double kp, double kd, double kp_ff)
    {
        kp_ = kp;
        kd_ = kd;
        if (kp_ff >= 0.0) kp_ff_ = kp_ff;
    }

}  // namespace RusRobotDriver