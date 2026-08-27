#pragma once

#include <mutex>

#include <Eigen/Dense>
#include <memory>

#include "EAIK/EAIK.h"
#include "components/types.hpp"

namespace RusRobotDriver {

    /** 模型末端(wrist3_link)到法兰的 Z 向偏移量 [m]（法奥 Fairino3_v6 实测值）
     *  注意：flange_offset 属于法兰坐标定义（wrist3_link + 偏移 = 法兰），
     *  不属于工具坐标系；工具坐标系（tool_transform_）纯粹表示「工具/探头相对法兰」。 */
    constexpr double kFlangeOffset = 0.0938;

    /**
     * @brief 运动学求解器
     *
     * 封装 EAIK 运动学模型，提供带工具坐标系变换补偿的正/逆运动学、
     * 数值 Jacobian 和 IK 选解功能。构造时注入模型，后续调用无需
     * 重复传入 ki_model 和 flange_offset。
     *
     * 工具坐标系（TCP 相对法兰）可运行时切换（SetToolTransform），
     * FK 输出 TCP 位姿，IK 输入 TCP 位姿；运动学内部自动应用工具变换。
     */
    class KinematicsSolver {
    public:
        /**
         * @param ki_model      EAIK 运动学模型
         * @param flange_offset 模型末端(wrist3_link)到法兰的 Z 向偏移 [m]（默认 kFlangeOffset）；
         *                      法兰坐标 = 模型末端 + 偏移，不并入工具变换
         */
        explicit KinematicsSolver(
            std::shared_ptr<const EAIK::Robot> ki_model,
            double flange_offset = kFlangeOffset);

        /**
         * @brief 正运动学：关节角 → TCP 位姿（含工具坐标系变换）
         *
         *   T_tcp = T_flange × T_tool
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
         * @brief 逆运动学：TCP 位姿 → 关节角解集（含工具坐标系反向补偿）
         *
         *   T_flange = T_tcp × inv(T_tool)
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

        /** 法兰偏移量访问器（模型末端到法兰的 Z 向偏移） */
        double flange_offset() const { return flange_offset_; }

        /**
         * @brief 设置工具坐标系变换（TCP 相对法兰），线程安全
         *
         * 切换工具坐标系后，后续 FK/IK 立即使用新工具变换。
         *
         * @param T 工具变换矩阵
         */
        void SetToolTransform(const Eigen::Matrix4d& T);

        /** 当前工具变换访问器 */
        Eigen::Matrix4d tool_transform() const;

    private:
        std::shared_ptr<const EAIK::Robot> ki_model_;
        double flange_offset_;
        mutable std::mutex mtx_;
        mutable Eigen::Matrix4d tool_transform_ = Eigen::Matrix4d::Identity();  // 工具坐标系变换矩阵（TCP 相对法兰）
    };

}  // namespace RusRobotDriver
