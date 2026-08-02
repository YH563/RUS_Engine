#pragma once

#include "controller/controller.hpp"

namespace RusRobotDriver {

    /**
     * @brief 仅重力补偿控制器
     *
     *  τ = bias(q, 0)  即仅补偿重力项，适用于低速/静态场景。
     */
    class GravityCompController : public IController {
    public:
        /**
         * @brief 构造重力补偿控制器
         *
         * @param dynamics 动力学回调
         */
        GravityCompController(DynamicsFunc dynamics);

        /**
         * @brief 计算重力补偿力矩
         */
        VectorXd ComputeTorque(const ControlTarget& target,
                               const RobotState& state) override;

    private:
        DynamicsFunc dynamics_;
    };

}  // namespace RusRobotDriver