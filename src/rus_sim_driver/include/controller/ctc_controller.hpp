#pragma once

#include "controller/controller.hpp"

namespace RusRobotDriver {

    /**
     * @brief 计算力矩控制（CTC）控制器
     *
     * 控制律：
     *   a_des = kp_ff · qdd_des + Kd · (qd_des - qd) + Kp · (q_des - q)
     *   τ     = M(q) · a_des + bias(q, qd)
     */
    class CtcController : public IController {
    public:
        /**
         * @brief 构造 CTC 控制器
         *
         * @param dynamics 动力学回调
         * @param kp       位置增益
         * @param kd       速度增益
         * @param kp_ff    前馈加速度增益
         */
        CtcController(DynamicsFunc dynamics,
                      double kp   = 400.0,
                      double kd   = 40.0,
                      double kp_ff = 1.0);

        /**
         * @brief 计算 CTC 控制力矩
         */
        VectorXd ComputeTorque(const ControlTarget& target,
                               const RobotState& state) override;

        /**
         * @brief 重置（CTC 无状态）
         */
        void Reset() override {}

        /**
         * @brief 在线调整增益
         *
         * @param kp    位置增益
         * @param kd    速度增益
         * @param kp_ff 前馈加速度增益（负值则不修改）
         */
        void SetGain(double kp, double kd, double kp_ff = -1.0);

    private:
        DynamicsFunc dynamics_;
        double kp_;
        double kd_;
        double kp_ff_;
    };

}  // namespace RusRobotDriver