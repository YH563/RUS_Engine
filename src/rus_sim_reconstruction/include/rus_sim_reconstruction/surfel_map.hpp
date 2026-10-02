#pragma once

// ════════════════════════════════════════════════════════════════════
//  CPU 面元地图（reconstruction：已知位姿的 surfel 融合）
//  ────────────────────────────────────────────────────────────────────
//  逐帧把「传感器系点 + 位姿」融合进一张体素哈希的面元地图：
//    - 每个体素内按位置 + 法线夹角匹配已有面元；
//    - 命中则用置信度加权的 EMA 更新位置 / 法线 / 颜色，置信度 +1；
//    - 未命中则新建面元（同体素内允许多个不同朝向的表面）。
//
//  已知机械臂位姿 → 无需 ICP/视觉里程计，这是本项目的天然优势。
//  纯 Eigen + std，无 ROS / 无 PCL，可脱离运行时单测。
//
//  性能：体素索引用扁平开放寻址哈希网格（voxel_hash.hpp）；匹配优先查
//  本体素（快路径），仅在本体素无近邻匹配时才回退查 3x3x3 邻域。
//
//  线程模型：非线程安全（由单一处理线程独占）。
// ════════════════════════════════════════════════════════════════════

#include <cstddef>
#include <cstdint>
#include <vector>

#include "rus_sim_reconstruction/types.hpp"
#include "rus_sim_reconstruction/voxel_hash.hpp"

namespace RusReconstruction {

    /// 体素哈希面元地图
    class SurfelMap {
    public:
        explicit SurfelMap(const SurfelFusionOptions& options = {});
        ~SurfelMap() = default;

        /// 更新融合选项（会清空地图）
        void SetOptions(const SurfelFusionOptions& options);
        const SurfelFusionOptions& Options() const { return options_; }

        /// 融合一帧（points 为空则直接返回）
        void Fuse(const Frame& frame);

        /// 只融合 frame 中指定的点下标子集（供分片并行融合用；单线程独占本对象）
        void FuseIndices(const Frame& frame, const uint32_t* indices, size_t count);

        /// 清空地图
        void Clear();

        /// 快照（拷贝，供发布 / 导出）
        /// @param min_confidence 只保留置信度 ≥ 此值的面元（0 = 全部）；
        ///        用于滤掉单次观测的离群碎面元。
        std::vector<Surfel> Snapshot(float min_confidence = 0.0f) const;

        /// 面元上限（0 = 不限）；达到上限后只更新、不再新增
        void SetMaxSurfels(size_t n) { max_surfels_ = n; }

        size_t SurfaceCount() const { return surfels_.size(); }
        size_t VoxelCount() const { return voxels_.Size(); }
        uint64_t PointsFused() const { return points_fused_; }
        uint64_t FramesFused() const { return frames_fused_; }

    private:
        VoxelKey KeyOf(const Vec3& p) const;

        /// 在 key 及其 3x3x3 邻域内找位置最近且法线兼容的面元（-1 = 无）。
        /// 快路径：本体素内有足够近的匹配则直接返回，不查邻域。
        int FindMatch(const VoxelKey& key, const Vec3& p, const Vec3& normal, bool has_normal,
                      double cos_thr, double max_dist) const;

        /// 融合单个点（p = R * sensor_pt + t）
        void FuseOne(const Vec3& p, const Vec3& normal, bool has_normal, uint32_t color,
                     double cos_thr, double max_dist, double stamp);

        SurfelFusionOptions options_;
        VoxelHash voxels_;
        std::vector<Surfel> surfels_;
        uint64_t points_fused_ = 0;
        uint64_t frames_fused_ = 0;
        size_t max_surfels_ = 0;
    };

}  // namespace RusReconstruction
