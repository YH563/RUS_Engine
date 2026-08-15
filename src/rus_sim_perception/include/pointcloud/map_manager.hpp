#pragma once

// ════════════════════════════════════════════════════════════════════
//  实时点云地图（pointcloud：点云算法子模块）
//  ────────────────────────────────────────────────────────────────────
//  感知层实时建图核心：逐帧累积点云 → 发布前整体体素化（去重叠冗余点 /
//  保密度）。点数超过上限自动触发体素化，保证内存有界。
//
//  纯算法实现：仅依赖 PCL / std，无 ROS 类型。
// ════════════════════════════════════════════════════════════════════

#include <cstddef>

#include "components/types.hpp"

namespace RusPerception::PointCloud {

    /// 实时点云地图（增量累积）
    class MapManager {
    public:
        /// 加入一帧（追加到地图，不立即去重）
        void AddFrame(const CloudRGBPtr& frame);

        /// 整体体素化（去重叠冗余点 + 降密度），发布前调用
        void Compact();

        /// 清空地图（map_clear 指令 / 换场景）
        void Clear();

        size_t FrameCount() const { return frame_count_; }
        size_t PointCount() const { return map_.size(); }

        /// 只读访问地图（非线程安全，仅供同线程使用）
        const CloudRGB& Map() const { return map_; }

        /// 拷贝快照（发布用，避免外部持有内部引用）
        CloudRGBPtr Snapshot() const;

        /// 点数上限（超过自动体素化；0 = 不限）
        void SetMaxPoints(size_t n) { max_points_ = n; }
        /// 体素化粒度（米）
        void SetVoxelLeafSize(float s) { voxel_leaf_size_ = s; }

    private:
        void voxelize();

        CloudRGB map_;
        size_t frame_count_ = 0;
        size_t max_points_ = 0;           // 0 = 不限制
        float voxel_leaf_size_ = 0.003f;  // 与滤波链体素一致
    };

}  // namespace RusPerception::PointCloud
