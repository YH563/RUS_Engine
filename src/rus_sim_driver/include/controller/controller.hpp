#pragma once

#include <memory>
#include <functional>

#include "components/types.hpp"

namespace RusRobotDriver {

    /**
     * @brief 控制器抽象基类
     *
     * 输入期望目标（ControlTarget）和实际状态（RobotState），
     * 输出关节力矩 VectorXd。可通过继承实现不同的控制律。
     */
    class IController {
    public:
        virtual ~IController() = default;

        /**
         * @brief 计算关节控制力矩
         *
         * @param target 期望控制目标
         * @param state  实际机器人状态
         * @return VectorXd 关节力矩 [Nm]
         */
        virtual VectorXd ComputeTorque(const ControlTarget& target,
                                       const RobotState& state) = 0;

        /**
         * @brief 重置控制器内部状态
         */
        virtual void Reset() {}
    };

    /**
     * @brief 动力学计算回调
     *
     * 给定关节位置 q 和速度 qd，计算惯性矩阵 M 和偏置力 bias。
     *
     * @param q    关节位置
     * @param qd   关节速度
     * @param M    输出：惯性矩阵 [n×n]
     * @param bias 输出：偏置力（科氏力 + 重力 + 摩擦力）
     */
    using DynamicsFunc = std::function<void(const VectorXd& q, const VectorXd& qd,
                                            Eigen::MatrixXd& M, VectorXd& bias)>;

    /**
     * @brief 控制器工厂
     *
     * 用法：
     *   auto ctc = ControllerFactory::Create(ControllerFactory::CTC, dynamics);
     */
    class ControllerFactory {
    public:
        enum Type { CTC, GravityComp };

        /**
         * @brief 创建控制器实例
         *
         * @param type     控制器类型
         * @param dynamics 动力学回调
         * @param kp       位置增益（CTC 使用）
         * @param kd       速度增益（CTC 使用）
         * @return std::unique_ptr<IController>
         */
        static std::unique_ptr<IController> Create(
            Type type,
            DynamicsFunc dynamics,
            double kp = 400.0,
            double kd = 40.0);
    };

}  // namespace RusRobotDriver
