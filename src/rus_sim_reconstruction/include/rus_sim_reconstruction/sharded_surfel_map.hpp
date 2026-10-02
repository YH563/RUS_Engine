#pragma once

// ════════════════════════════════════════════════════════════════════
//  分片并行面元地图（reconstruction）
//  ────────────────────────────────────────────────────────────────────
//  把体素空间按「体素键哈希 % shards」分到 N 个独立子地图（SurfelMap），
//  每帧的点按所属分片分桶后，由线程池并行融合。因为一个体素固定属于一个
//  分片、每个分片同一时刻只被一个线程处理 → 无数据竞争，无需锁。
//
//  代价：跨分片的邻域匹配会缺失（分片按哈希随机，边界零散），对质量影响很小；
//  收益：多核并行，全分辨率点云可近实时。
// ════════════════════════════════════════════════════════════════════

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include "rus_sim_reconstruction/surfel_map.hpp"
#include "rus_sim_reconstruction/thread_pool.hpp"
#include "rus_sim_reconstruction/voxel_hash.hpp"

namespace RusReconstruction {

    class ShardedSurfelMap {
    public:
        /// @param shards  分片数（= 并行度上界）
        /// @param threads 线程数（0 = 硬件并发；通常 = shards）
        explicit ShardedSurfelMap(const SurfelFusionOptions& options = {},
                                  int shards = 8, unsigned threads = 0);
        ~ShardedSurfelMap() = default;

        void SetOptions(const SurfelFusionOptions& options);
        const SurfelFusionOptions& Options() const { return options_; }

        void Fuse(const Frame& frame);
        void Clear();

        std::vector<Surfel> Snapshot(float min_confidence = 0.0f) const;

        size_t SurfaceCount() const;
        size_t VoxelCount() const;
        uint64_t PointsFused() const;
        int Shards() const { return static_cast<int>(shards_.size()); }

    private:
        SurfelFusionOptions options_;
        std::vector<std::unique_ptr<SurfelMap>> shards_;
        std::vector<std::vector<uint32_t>> buckets_;  // 每分片的点下标（复用）
        ThreadPool pool_;
    };

}  // namespace RusReconstruction
