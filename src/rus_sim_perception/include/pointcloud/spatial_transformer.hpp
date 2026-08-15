#pragma once

// ════════════════════════════════════════════════════════════════════
//  空间变换器（pointcloud：点云算法子模块）
//  ────────────────────────────────────────────────────────────────────
//  眼在手上标定：相机坐标系点云 → base_link 坐标系。
//    T_base_camera = T_base_flange * camera_to_flange
//  其中 T_base_flange 由 PoseInterpolator 按点云时间戳插值得到。
//
//  纯算法实现：仅依赖 Eigen / PCL，无 ROS 类型。
// ════════════════════════════════════════════════════════════════════

#include <Eigen/Geometry>

#include "components/types.hpp"

namespace RusPerception::PointCloud {

    /// 相机坐标系 → base_link 变换器
    class SpatialTransformer {
    public:
        /// 设置 相机→法兰 标定矩阵（未标定保持单位阵，相机与法兰重合）
        void SetCameraToFlange(const Eigen::Matrix4f& T) { camera_to_flange_ = T; }
        const Eigen::Matrix4f& CameraToFlange() const { return camera_to_flange_; }

        /**
         * @brief 变换一帧点云到 base_link
         *
         * @param in            相机坐标系点云
         * @param T_base_flange 该帧对应的法兰位姿（base_link 系）
         * @param out           输出 base_link 系点云（新分配）
         * @return true 成功；false 输入为空
         */
        bool Transform(const CloudRGBPtr& in,
                       const Eigen::Matrix4f& T_base_flange,
                       CloudRGBPtr& out) const;

    private:
        Eigen::Matrix4f camera_to_flange_ = Eigen::Matrix4f::Identity();
    };

}  // namespace RusPerception::PointCloud
