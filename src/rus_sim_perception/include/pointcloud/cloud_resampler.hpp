#pragma once

// ════════════════════════════════════════════════════════════════════
//  点云均匀重采样（pointcloud：点云算法子模块）
//  ────────────────────────────────────────────────────────────────────
//  目标：降密 + 点间距均匀 + 表面平滑 + 法线，改善**轨迹生成**
//  （k-NN 建图 / Gabriel 稳定）与**重建 / 点云图增量更新**的输入质量。
//
//  算法链：
//    VoxelGrid（粗降密）→ StatisticalOutlierRemoval（去离群）
//    → MLS（MovingLeastSquares, NONE：平滑 + 法线）
//    → 网格贪心均匀采样（保证任意两点间距 ≥ target，近似 Poisson-disk）
//
//  ⚠️ 降密 ≠ 去噪：要平滑必须开 MLS。`target_spacing` 需 ≥ 噪声量级，且足够大
//     以支撑下游建图（planning 的 graph_k）。
//
//  纯算法实现：仅依赖 PCL / Eigen，无 ROS 类型。
// ════════════════════════════════════════════════════════════════════

#include <cstddef>
#include <vector>

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "components/types.hpp"

namespace RusPerception::PointCloud {

    /// 重采样选项
    struct ResampleOptions {
        bool  enable_voxel = true;          // 粗降密
        float voxel_leaf = 0.006f;          // 体素边长（≤0 跳过）
        bool  enable_sor = true;            // 统计离群去除
        int   sor_mean_k = 20;
        float sor_std = 1.0f;
        bool  enable_mls = true;            // MLS 平滑 + 法线
        float mls_search_radius = 0.012f;   // 局部拟合半径（≈2~3×目标间距）
        float mls_target_spacing = 0.004f;  // 目标点间距（均匀采样步长）
        int   mls_order = 2;                // 多项式阶数
    };

    /// 重采样结果
    struct ResampleResult {
        CloudRGB cloud;                       // 重采样后位置 + 颜色
        std::vector<Eigen::Vector3f> normals; // 单位法线（与 cloud 一一对应；无则空）
    };

    /// 重采样（输入不变，输出到 result）
    void Resample(const CloudRGB& in, ResampleResult& out, const ResampleOptions& opt);

    /// 最近邻间距统计（k=2，取最近的非自身点）
    struct SpacingStats {
        size_t n = 0;
        float mean = 0.f, median = 0.f, p5 = 0.f, p95 = 0.f, min = 0.f, max = 0.f;
    };
    SpacingStats ComputeSpacingStats(const CloudRGB& cloud);

}  // namespace RusPerception::PointCloud
