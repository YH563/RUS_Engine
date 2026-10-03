#pragma once

// ════════════════════════════════════════════════════════════════════
//  点云均匀重采样（预处理工具）
//  ────────────────────────────────────────────────────────────────────
//  目标：降密 + 点间距均匀 + 表面平滑 + 法线，改善轨迹生成与重建输入。
//
//  算法链：
//    VoxelGrid（粗降密）→ StatisticalOutlierRemoval（去离群）
//    → MLS（MovingLeastSquares, SAMPLE_LOCAL_PLANE）：在平滑表面上以
//      目标间距重新撒点并给法线。
//
//  依赖 PCL（filters / surface / search / kdtree）。
// ════════════════════════════════════════════════════════════════════

#include <cstddef>
#include <vector>

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace RusReconstruction {

    using CloudRGB = pcl::PointCloud<pcl::PointXYZRGB>;

    struct ResampleOptions {
        bool  enable_voxel = true;          // 粗降密
        float voxel_leaf = 0.006f;          // 体素边长（≤0 跳过）
        bool  enable_sor = true;            // 统计离群去除
        int   sor_mean_k = 20;
        float sor_std = 1.0f;
        bool  enable_mls = true;            // MLS 平滑 + 均匀重采样 + 法线
        float mls_search_radius = 0.012f;   // 局部拟合半径（≈2~3×目标间距）
        float mls_target_spacing = 0.004f;  // 目标点间距（重采样步长）
        int   mls_order = 2;                // 多项式阶数
    };

    struct ResampleResult {
        CloudRGB cloud;                              // 重采样后位置 + 颜色
        std::vector<Eigen::Vector3f> normals;        // 单位法线（与 cloud 一一对应；无则空）
    };

    /// 重采样（原地不动，输出到 result）
    void Resample(const CloudRGB& in, ResampleResult& out, const ResampleOptions& opt);

    /// 最近邻间距统计（k=2，取最近的非自身点）
    struct SpacingStats {
        size_t n = 0;
        float mean = 0.f, median = 0.f, p5 = 0.f, p95 = 0.f, min = 0.f, max = 0.f;
    };
    SpacingStats ComputeSpacingStats(const CloudRGB& cloud);

}  // namespace RusReconstruction
