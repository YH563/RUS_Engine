#include "rus_sim_reconstruction/surfel_map.hpp"

#include <algorithm>
#include <cmath>

namespace RusReconstruction {

    namespace {
        constexpr double kPi = 3.14159265358979323846;
        // 位置匹配半径相对体素的默认比例（<1，避免把相邻网格点误合并）
        constexpr double kMaxDistanceRatio = 0.9;
    }

    SurfelMap::SurfelMap(const SurfelFusionOptions& options) : options_(options) {}

    void SurfelMap::SetOptions(const SurfelFusionOptions& options)
    {
        options_ = options;
        // 体素尺寸变化会让已有键失效 → 直接清空，避免残留错误聚合
        Clear();
    }

    VoxelKey SurfelMap::KeyOf(const Vec3& p) const
    {
        const double inv = 1.0 / options_.voxel_size;
        VoxelKey k;
        k.x = static_cast<int32_t>(std::floor(p.x() * inv));
        k.y = static_cast<int32_t>(std::floor(p.y() * inv));
        k.z = static_cast<int32_t>(std::floor(p.z() * inv));
        return k;
    }

    namespace {

        // 扫描一个体素桶，更新最近且法线兼容的面元
        inline void scan_bucket(const VoxelHash::Bucket* b, const std::vector<Surfel>& surfels,
                                const Vec3& p, const Vec3& normal, bool has_normal,
                                double cos_thr, int& best, double& best_dist)
        {
            for (int i = 0; i < b->count; ++i) {
                const int idx = b->idx[i];
                const Surfel& s = surfels[static_cast<size_t>(idx)];
                const double d = (s.position - p).norm();
                if (d > best_dist) continue;
                if (has_normal && s.confidence > 0.0f && normal.dot(s.normal) < cos_thr) continue;
                best_dist = d;
                best = idx;
            }
        }

    }  // namespace

    int SurfelMap::FindMatch(const VoxelKey& key, const Vec3& p, const Vec3& normal,
                             bool has_normal, double cos_thr, double max_dist) const
    {
        int best = -1;
        double best_dist = max_dist;

        // 快路径：本体素只要有法线兼容的匹配即返回（稳定后绝大多数走这里）。
        // 注意：只有「本体素+邻域都无匹配」才会新建面元，因此不会产生边界重影。
        if (const VoxelHash::Bucket* b = voxels_.Find(key)) {
            scan_bucket(b, surfels_, p, normal, has_normal, cos_thr, best, best_dist);
            if (best >= 0) return best;
        }

        // 回退：查 3x3x3 邻域（跳过已查过的本体素）
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dz = -1; dz <= 1; ++dz) {
                    if (dx == 0 && dy == 0 && dz == 0) continue;
                    const VoxelKey k{key.x + dx, key.y + dy, key.z + dz};
                    if (const VoxelHash::Bucket* b = voxels_.Find(k)) {
                        scan_bucket(b, surfels_, p, normal, has_normal, cos_thr, best, best_dist);
                    }
                }
            }
        }
        return best;
    }

    void SurfelMap::FuseOne(const Vec3& p, const Vec3& normal, bool has_normal, uint32_t col,
                            double cos_thr, double max_dist, double stamp)
    {
        const VoxelKey key = KeyOf(p);
        const int best = FindMatch(key, p, normal, has_normal, cos_thr, max_dist);

        if (best >= 0) {
            Surfel& s = surfels_[static_cast<size_t>(best)];
            const double w = static_cast<double>(s.confidence) /
                             (static_cast<double>(s.confidence) + 1.0);
            s.position = w * s.position + (1.0 - w) * p;
            if (has_normal) {
                Vec3 nn = w * s.normal + (1.0 - w) * normal;
                if (nn.norm() > 1e-9) s.normal = nn.normalized();
            }
            if (col != 0) s.color = col;
            s.confidence += 1.0f;
            s.last_stamp = stamp;
        } else {
            if (max_surfels_ != 0 && surfels_.size() >= max_surfels_) return;
            VoxelHash::Bucket& b = voxels_.GetOrCreate(key);
            if (b.count >= options_.max_surfels_per_voxel || b.count >= VoxelHash::kBucketCap) return;
            Surfel s;
            s.position = p;
            s.normal = has_normal ? normal : Vec3::UnitZ();
            s.radius = static_cast<float>(options_.voxel_size);
            s.confidence = 1.0f;
            s.color = col;
            s.last_stamp = stamp;
            surfels_.push_back(s);
            b.idx[b.count++] = static_cast<int>(surfels_.size() - 1);
        }
        ++points_fused_;
    }

    void SurfelMap::Fuse(const Frame& frame)
    {
        if (frame.points.empty()) return;

        const size_t n = frame.points.size();
        const bool has_normals = frame.normals.size() == n;
        const bool has_colors = frame.colors.size() == n;

        const Mat4& T = frame.T_base_sensor;
        const Eigen::Matrix3d R = T.block<3, 3>(0, 0);
        const Vec3 t = T.block<3, 1>(0, 3);

        const double cos_thr = std::cos(options_.max_normal_angle_deg * kPi / 180.0);
        double max_dist = options_.max_distance > 0.0 ? options_.max_distance
                                                      : kMaxDistanceRatio * options_.voxel_size;
        max_dist = std::min(max_dist, kMaxDistanceRatio * options_.voxel_size);

        for (size_t i = 0; i < n; ++i) {
            const Vec3 p = R * frame.points[i] + t;
            bool has_n = has_normals && frame.normals[i].norm() > 1e-9;
            Vec3 nrm = Vec3::Zero();
            if (has_n) {
                nrm = R * frame.normals[i];
                if (nrm.norm() > 1e-9) nrm.normalize();
            }
            const uint32_t col = has_colors ? frame.colors[i] : 0;
            FuseOne(p, nrm, has_n, col, cos_thr, max_dist, frame.stamp);
        }
        ++frames_fused_;
    }

    void SurfelMap::FuseIndices(const Frame& frame, const uint32_t* indices, size_t count)
    {
        if (count == 0 || frame.points.empty()) return;

        const size_t n = frame.points.size();
        const bool has_normals = frame.normals.size() == n;
        const bool has_colors = frame.colors.size() == n;

        const Mat4& T = frame.T_base_sensor;
        const Eigen::Matrix3d R = T.block<3, 3>(0, 0);
        const Vec3 t = T.block<3, 1>(0, 3);

        const double cos_thr = std::cos(options_.max_normal_angle_deg * kPi / 180.0);
        double max_dist = options_.max_distance > 0.0 ? options_.max_distance
                                                      : kMaxDistanceRatio * options_.voxel_size;
        max_dist = std::min(max_dist, kMaxDistanceRatio * options_.voxel_size);

        for (size_t k = 0; k < count; ++k) {
            const size_t i = indices[k];
            if (i >= n) continue;
            const Vec3 p = R * frame.points[i] + t;
            bool has_n = has_normals && frame.normals[i].norm() > 1e-9;
            Vec3 nrm = Vec3::Zero();
            if (has_n) {
                nrm = R * frame.normals[i];
                if (nrm.norm() > 1e-9) nrm.normalize();
            }
            const uint32_t col = has_colors ? frame.colors[i] : 0;
            FuseOne(p, nrm, has_n, col, cos_thr, max_dist, frame.stamp);
        }
        ++frames_fused_;
    }

    void SurfelMap::Clear()
    {
        voxels_.Clear();
        surfels_.clear();
        points_fused_ = 0;
        frames_fused_ = 0;
    }

    std::vector<Surfel> SurfelMap::Snapshot(float min_confidence) const
    {
        if (min_confidence <= 0.0f) return surfels_;
        std::vector<Surfel> out;
        out.reserve(surfels_.size());
        for (const auto& s : surfels_) {
            if (s.confidence >= min_confidence) out.push_back(s);
        }
        return out;
    }

}  // namespace RusReconstruction
