#pragma once

#include <Eigen/Dense>
#include <memory>

#include "EAIK/EAIK.h"
#include "components/types.hpp"

namespace RusRobotDriver {

    /** 法兰到 TCP 的 Z 向偏移量 [m]（法奥 Fairino3_v6 实测值） */
    constexpr double kFlangeOffset = 0.0938;

    /**
     * @brief 运动学求解器
     *
     * 封装 EAIK 运动学模型，提供带法兰偏移补偿的正/逆运动学、
     * 数值 Jacobian 和 IK 选解功能。构造时注入模型，后续调用无需
     * 重复传入 ki_model 和 flange_offset。
     */
    class KinematicsSolver {
    public:
        /**
         * @param ki_model      EAIK 运动学模型
         * @param flange_offset 法兰→TCP Z 向偏移 [m]（默认 kFlangeOffset）
         */
        explicit KinematicsSolver(
            std::shared_ptr<const EAIK::Robot> ki_model,
            double flange_offset = kFlangeOffset);

        /**
         * @brief 正运动学：关节角 → TCP 位姿（含法兰偏移补偿）
         *
         *   p_tcp = p_flange + offset * R(:,2)
         */
        Eigen::Matrix4d ForwardKinematics(const Eigen::VectorXd& joint_pos) const;

        /**
         * @brief 数值几何 Jacobian [6×n]
         *
         * 每个关节 i 扰动 ε 后 FK 差分：
         *   J_lin[:,i] = (p₁ - p₀) / ε
         *   J_ang[:,i] = axis_angle(R₀ᵀ·R₁) / ε
         */
        Eigen::MatrixXd NumericalJacobian(const Eigen::VectorXd& q) const;

        /**
         * @brief 逆运动学：TCP 位姿 → 关节角解集（含法兰偏移反向补偿）
         */
        IKS::IK_Solution InverseKinematics(const Eigen::Matrix4d& pose) const;

        /**
         * @brief 从 IK 多解中选距参考关节角最近的解（含角度环绕归一化）
         *
         * @param ik    IK 解集
         * @param ref   参考关节角
         * @param q_out 输出：选中的关节角
         * @return int  解索引，-1 无有效解
         */
        int PickBestIK(
            const IKS::IK_Solution& ik,
            const Eigen::VectorXd& ref,
            Eigen::VectorXd& q_out) const;

        /** 法兰偏移量访问器 */
        double flange_offset() const { return flange_offset_; }

    private:
        std::shared_ptr<const EAIK::Robot> ki_model_;
        double flange_offset_;
    };

}  // namespace RusRobotDriver
