#include "rus_sim_reconstruction/sharded_surfel_map.hpp"

#include <algorithm>
#include <cmath>

namespace RusReconstruction {

    namespace {
        // 按空间块分片：块内邻域完整，仅块边界的少量跨块匹配丢失（远好于哈希分片）
        inline size_t shard_of(const Vec3& p, double block, size_t ns)
        {
            const int32_t bx = static_cast<int32_t>(std::floor(p.x() / block));
            const int32_t by = static_cast<int32_t>(std::floor(p.y() / block));
            const int32_t bz = static_cast<int32_t>(std::floor(p.z() / block));
            const uint32_t h = static_cast<uint32_t>(bx) * 73856093u ^
                               static_cast<uint32_t>(by) * 19349663u ^
                               static_cast<uint32_t>(bz) * 83492791u;
            return h % ns;
        }
    }

    ShardedSurfelMap::ShardedSurfelMap(const SurfelFusionOptions& options, int shards, unsigned threads)
        : options_(options), pool_(threads)
    {
        if (shards < 1) shards = 1;
        shards_.reserve(shards);
        buckets_.resize(shards);
        for (int i = 0; i < shards; ++i) {
            shards_.emplace_back(std::make_unique<SurfelMap>(options));
        }
    }

    void ShardedSurfelMap::SetOptions(const SurfelFusionOptions& options)
    {
        options_ = options;
        for (auto& s : shards_) s->SetOptions(options);
    }

    void ShardedSurfelMap::Fuse(const Frame& frame)
    {
        if (frame.points.empty()) return;

        const size_t ns = shards_.size();
        for (auto& b : buckets_) b.clear();

        // 按「体素键哈希 % 分片数」分桶（串行；纯整数运算，开销小）
        const Mat4& T = frame.T_base_sensor;
        const Eigen::Matrix3d R = T.block<3, 3>(0, 0);
        const Vec3 t = T.block<3, 1>(0, 3);
        const double block = std::max(options_.voxel_size * 16.0, 0.05);
        const size_t n = frame.points.size();
        for (size_t i = 0; i < n; ++i) {
            const Vec3 p = R * frame.points[i] + t;
            buckets_[shard_of(p, block, ns)].push_back(static_cast<uint32_t>(i));
        }

        // 每分片由一个线程处理（分片间无共享 → 无锁）
        pool_.ParallelFor(ns, 1, [this, &frame](size_t b, size_t e) {
            for (size_t s = b; s < e; ++s) {
                shards_[s]->FuseIndices(frame, buckets_[s].data(), buckets_[s].size());
            }
        });
    }

    void ShardedSurfelMap::Clear()
    {
        for (auto& s : shards_) s->Clear();
    }

    std::vector<Surfel> ShardedSurfelMap::Snapshot(float min_confidence) const
    {
        std::vector<Surfel> out;
        for (const auto& s : shards_) {
            auto part = s->Snapshot(min_confidence);
            out.insert(out.end(), part.begin(), part.end());
        }
        return out;
    }

    size_t ShardedSurfelMap::SurfaceCount() const
    {
        size_t total = 0;
        for (const auto& s : shards_) total += s->SurfaceCount();
        return total;
    }

    size_t ShardedSurfelMap::VoxelCount() const
    {
        size_t total = 0;
        for (const auto& s : shards_) total += s->VoxelCount();
        return total;
    }

    uint64_t ShardedSurfelMap::PointsFused() const
    {
        uint64_t total = 0;
        for (const auto& s : shards_) total += s->PointsFused();
        return total;
    }

}  // namespace RusReconstruction
