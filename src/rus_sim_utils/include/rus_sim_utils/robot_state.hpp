#pragma once

#include <Eigen/Dense>

namespace RusUtils {
    using Eigen::VectorXd;

    /**
     * @brief 期望控制目标（轨迹调度器输出）
     */
    struct ControlTarget {
        VectorXd q_des;   // 期望关节位置 [rad]
        VectorXd qd_des;  // 期望关节速度 [rad/s]
        VectorXd qdd_des; // 期望关节加速度 [rad/s²]
    };

    /**
     * @brief 机器人状态（通用数据，无 SDK 依赖）
     */
    struct RobotState {
        VectorXd flange_pos;  // 法兰位姿 XYZABC [m/rad]
        VectorXd joint_pos;   // 关节位置 [rad]
        VectorXd joint_vel;   // 关节速度 [rad/s]
        VectorXd joint_acc;   // 关节加速度 [rad/s²]
        VectorXd effort;      // 关节力矩 [Nm]
        double timestamp = 0.0;

        // 工具坐标系信息
        int tool_index = 0;   // 当前工具坐标系索引 [0~14]，0 表示法兰坐标系
        VectorXd tool_pose;   // 当前 TCP 位姿（基坐标系下）XYZABC [m/rad]
    };
}
