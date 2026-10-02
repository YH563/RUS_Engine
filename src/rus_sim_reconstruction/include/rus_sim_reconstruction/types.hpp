#pragma once

// ════════════════════════════════════════════════════════════════════
//  重建层公共类型（reconstruction：面元地图的输入帧与面元定义）
//  ────────────────────────────────────────────────────────────────────
//  仅数据类型，无逻辑。纯 Eigen 依赖，便于脱离 ROS / PCL 单测。
// ════════════════════════════════════════════════════════════════════

#include <cstdint>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace RusReconstruction {

    using Vec3 = Eigen::Vector3d;   // 3D 向量
    using Mat4 = Eigen::Matrix4d;   // 4x4 齐次变换

    /// 单帧输入（传感器系点 + 可选法线/颜色 + 该帧到 base 的位姿）
    ///
    /// - points ：传感器系坐标（米）
    /// - normals：传感器系单位法线（可空 → 该帧跳过法线融合）
    /// - colors ：打包 RGB（r<<16|g<<8|b，可空）
    /// - T_base_sensor：传感器系 → base 系（通常来自 /driver/state 插值 + 手眼标定）
    struct Frame {
        std::vector<Vec3> points;
        std::vector<Vec3> normals;
        std::vector<uint32_t> colors;
        Mat4 T_base_sensor = Mat4::Identity();
        double stamp = 0.0;
    };

    /// 面元（base 系，带朝向的表面元素）
    struct Surfel {
        Vec3 position = Vec3::Zero();        // 面元中心
        Vec3 normal = Vec3::UnitZ();         // 单位法线
        float radius = 0.0f;                 // 面元半径（近似）
        float confidence = 0.0f;             // 累积观测次数（越大越可信）
        uint32_t color = 0;                  // 打包 RGB
        double last_stamp = 0.0;             // 最近一次观测时间
    };

    /// 融合选项
    struct SurfelFusionOptions {
        double voxel_size = 0.01;            // 体素边长（米）
        double max_normal_angle_deg = 30.0;  // 合并面元的法线夹角上限（超过则视为不同表面）
        double max_distance = 0.0;           // 位置匹配半径（米）；0 = 自动取 0.9*voxel_size，
                                             // 且内部钳制不超过 0.9*voxel_size（防止跨点误合并）
        int    max_surfels_per_voxel = 4;    // 单个体素最多容纳面元数（多表面）
    };

}  // namespace RusReconstruction
