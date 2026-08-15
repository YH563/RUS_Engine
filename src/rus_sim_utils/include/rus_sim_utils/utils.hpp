#pragma once

// ════════════════════════════════════════════════════════════════════
//  位姿 / 坐标通用转换工具（几何域）
//  ────────────────────────────────────────────────────────────────────
//  统一收口感知 / 规划等各模块共用的坐标、位姿转换，避免各包重复实现：
//    - PoseToMatrix4d / Matrix4dToPose   位姿 ↔ 齐次矩阵
//    - MakePose       坐标数组 → 位姿（外部指令 set_start_pose / set_end_pose）
//    - PoseToRPY      位姿 → RPY（下发驱动 servo_cart 用，固定轴 XYZ）
//    - FlangePosToPose  驱动状态 flange_pos[6] → 法兰位姿
//    - FlangeToProbe / ProbeToFlange   法兰 ↔ 探头（探头安装在机械臂末端，待标定）
//
//  坐标约定：m + rad；RPY 为固定轴 XYZ（R = Rz·Ry·Rx），
//  与驱动 flange_pos 输出 / servo_cart 输入一致。
//  依赖：Eigen、geometry_msgs
// ════════════════════════════════════════════════════════════════════

#include <cmath>
#include <vector>

#include <Eigen/Dense>
#include <geometry_msgs/msg/pose.hpp>

namespace RusUtils {

    /// 位姿 → 4x4 齐次变换矩阵
    inline Eigen::Matrix4d PoseToMatrix4d(const geometry_msgs::msg::Pose& pose)
    {
        Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
        Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x,
                             pose.orientation.y, pose.orientation.z);
        mat.block<3, 3>(0, 0) = q.toRotationMatrix();
        mat.block<3, 1>(0, 3) = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
        return mat;
    }

    /// 4x4 齐次变换矩阵 → 位姿
    inline geometry_msgs::msg::Pose Matrix4dToPose(const Eigen::Matrix4d& mat)
    {
        geometry_msgs::msg::Pose pose;
        pose.position.x = mat(0, 3);
        pose.position.y = mat(1, 3);
        pose.position.z = mat(2, 3);
        Eigen::Quaterniond q(mat.block<3, 3>(0, 0));
        pose.orientation.w = q.w();
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        return pose;
    }

    /**
     * @brief 坐标数组 → 位姿（外部指令 set_start_pose / set_end_pose 用，单位 m/rad）
     *
     * 支持三种格式：
     *   [x,y,z]                    —— 仅位置，姿态默认单位四元数
     *   [x,y,z,rx,ry,rz]           —— 位置 + RPY（固定轴 XYZ）
     *   [x,y,z,qx,qy,qz,qw]        —— 位置 + 四元数
     *
     * @param v 坐标数组
     * @return 构造的位姿
     */
    inline geometry_msgs::msg::Pose MakePose(const std::vector<double>& v)
    {
        geometry_msgs::msg::Pose p;
        if (v.size() >= 1) p.position.x = v[0];
        if (v.size() >= 2) p.position.y = v[1];
        if (v.size() >= 3) p.position.z = v[2];
        if (v.size() >= 7) {  // 位置 + 四元数 xyzw
            p.orientation.x = v[3];
            p.orientation.y = v[4];
            p.orientation.z = v[5];
            p.orientation.w = v[6];
        } else if (v.size() >= 6) {  // 位置 + RPY → 矩阵 → 位姿
            double rx = v[3], ry = v[4], rz = v[5];
            Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
            T.block<3, 3>(0, 0) =
                (Eigen::AngleAxisd(rz, Eigen::Vector3d::UnitZ())
               * Eigen::AngleAxisd(ry, Eigen::Vector3d::UnitY())
               * Eigen::AngleAxisd(rx, Eigen::Vector3d::UnitX())).toRotationMatrix();
            T.block<3, 1>(0, 3) << v[0], v[1], v[2];
            p = Matrix4dToPose(T);
        } else {  // 仅位置，姿态默认单位
            p.orientation.w = 1.0;
        }
        return p;
    }

    /**
     * @brief 位姿 → RPY（固定轴 XYZ，R = Rz·Ry·Rx，与驱动 servo_cart 约定一致）
     *
     * @param pose    输入位姿
     * @param rx, ry, rz  输出 RPY [rad]
     */
    inline void PoseToRPY(const geometry_msgs::msg::Pose& pose,
                          double& rx, double& ry, double& rz)
    {
        Eigen::Matrix3d R = PoseToMatrix4d(pose).block<3, 3>(0, 0);
        rx = std::atan2(R(2, 1), R(2, 2));
        ry = std::asin(-R(2, 0));
        rz = std::atan2(R(1, 0), R(0, 0));
    }

    /// 驱动状态 flange_pos[6]（[x,y,z,rx,ry,rz]，m+rad，固定轴 XYZ RPY）→ 法兰位姿
    inline geometry_msgs::msg::Pose FlangePosToPose(const std::vector<double>& flange_pos)
    {
        geometry_msgs::msg::Pose pose;
        if (flange_pos.size() >= 3) {
            pose.position.x = flange_pos[0];
            pose.position.y = flange_pos[1];
            pose.position.z = flange_pos[2];
        }
        if (flange_pos.size() >= 6) {
            const double rx = flange_pos[3];
            const double ry = flange_pos[4];
            const double rz = flange_pos[5];
            Eigen::AngleAxisd roll(rx, Eigen::Vector3d::UnitX());
            Eigen::AngleAxisd pitch(ry, Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd yaw(rz, Eigen::Vector3d::UnitZ());
            Eigen::Quaterniond q = yaw * pitch * roll;
            pose.orientation.x = q.x();
            pose.orientation.y = q.y();
            pose.orientation.z = q.z();
            pose.orientation.w = q.w();
        } else {
            pose.orientation.w = 1.0;
        }
        return pose;
    }

    /**
     * @brief 法兰位姿 → 探头位姿
     *
     * 探头安装在机械臂末端，probe_to_flange 为 法兰→探头 的变换矩阵（待标定）。
     *
     * @param pose            法兰位姿
     * @param probe_to_flange 法兰→探头 变换矩阵
     * @return 探头位姿
     */
    inline geometry_msgs::msg::Pose FlangeToProbe(
        const geometry_msgs::msg::Pose& pose,
        const Eigen::Matrix4d& probe_to_flange)
    {
        return Matrix4dToPose(PoseToMatrix4d(pose) * probe_to_flange);
    }

    /**
     * @brief 探头位姿 → 法兰位姿
     *
     * probe_to_flange 为 法兰→探头 的变换矩阵（待标定）。
     *
     * @param pose            探头位姿
     * @param probe_to_flange 法兰→探头 变换矩阵
     * @return 法兰位姿
     */
    inline geometry_msgs::msg::Pose ProbeToFlange(
        const geometry_msgs::msg::Pose& pose,
        const Eigen::Matrix4d& probe_to_flange)
    {
        return Matrix4dToPose(PoseToMatrix4d(pose) * probe_to_flange.inverse());
    }

}  // namespace RusUtils
