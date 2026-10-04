#include "rus_sim_reconstruction/tsdf_volume.hpp"

#include <algorithm>
#include <cmath>

namespace RusReconstruction {

    namespace {
        // 立方体 8 角的体素偏移（0/1）
        constexpr int kCorner[8][3] = {
            {0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {1, 1, 0},
            {0, 0, 1}, {1, 0, 1}, {0, 1, 1}, {1, 1, 1}
        };
        // 立方体剖分为 6 个四面体（共享**体对角线** 0-7；环 1-3-2-6-4-5）。
        // 之前误用面对角线 0-6 导致没铺满立方体、相邻块拼不上 → 满屏孔洞。
        constexpr int kTets[6][4] = {
            {0, 1, 3, 7}, {0, 3, 2, 7}, {0, 2, 6, 7},
            {0, 6, 4, 7}, {0, 4, 5, 7}, {0, 5, 1, 7}
        };
    }

    TsdfVolume::TsdfVolume(const Options& options)
        : opt_(options), bs_(options.block_size), trunc_(options.truncation)
    {
        if (bs_ != 8) bs_ = 8;  // 块固定 8^3（Block 数组尺寸）
    }

    int TsdfVolume::floor_div(int a, int b)
    {
        int q = a / b;
        if ((a % b != 0) && ((a < 0) != (b < 0))) --q;
        return q;
    }

    TsdfVolume::BlockKey TsdfVolume::BlockOfVoxel(int vx, int vy, int vz,
                                                  int& lx, int& ly, int& lz) const
    {
        BlockKey k;
        k.x = floor_div(vx, bs_);
        k.y = floor_div(vy, bs_);
        k.z = floor_div(vz, bs_);
        lx = vx - k.x * bs_;
        ly = vy - k.y * bs_;
        lz = vz - k.z * bs_;
        return k;
    }

    Vec3 TsdfVolume::ChunkOrigin(const BlockKey& k) const
    {
        const double s = bs_ * opt_.voxel_size;
        return Vec3(k.x * s, k.y * s, k.z * s);
    }

    int64_t TsdfVolume::PackChunkId(int32_t x, int32_t y, int32_t z)
    {
        const uint64_t ux = static_cast<uint32_t>(x) & 0x1FFFFFu;
        const uint64_t uy = static_cast<uint32_t>(y) & 0x1FFFFFu;
        const uint64_t uz = static_cast<uint32_t>(z) & 0x1FFFFFu;
        return static_cast<int64_t>((ux << 42) | (uy << 21) | uz);
    }

    void TsdfVolume::UnpackChunkId(int64_t id, int32_t& x, int32_t& y, int32_t& z)
    {
        auto sext = [](uint32_t v) -> int32_t {
            v &= 0x1FFFFFu;
            return (v & 0x100000u) ? static_cast<int32_t>(v | 0xFFE00000u)
                                   : static_cast<int32_t>(v);
        };
        x = sext(static_cast<uint32_t>((static_cast<uint64_t>(id) >> 42) & 0x1FFFFFu));
        y = sext(static_cast<uint32_t>((static_cast<uint64_t>(id) >> 21) & 0x1FFFFFu));
        z = sext(static_cast<uint32_t>(static_cast<uint64_t>(id) & 0x1FFFFFu));
    }

    void TsdfVolume::MarkDirtyAround(const BlockKey& k)
    {
        for (int dx = -1; dx <= 1; ++dx)
            for (int dy = -1; dy <= 1; ++dy)
                for (int dz = -1; dz <= 1; ++dz)
                    dirty_.insert(BlockKey{k.x + dx, k.y + dy, k.z + dz});
    }

    void TsdfVolume::UpdateVoxel(const Vec3& pos, float sdf, float w)
    {
        const double inv = 1.0 / opt_.voxel_size;
        const int vx = static_cast<int>(std::floor(pos.x() * inv));
        const int vy = static_cast<int>(std::floor(pos.y() * inv));
        const int vz = static_cast<int>(std::floor(pos.z() * inv));

        int lx, ly, lz;
        const BlockKey key = BlockOfVoxel(vx, vy, vz, lx, ly, lz);
        Block& b = blocks_[key];
        const int idx = (lz * bs_ + ly) * bs_ + lx;

        const float old_w = b.weight[idx];
        const float new_w = std::min<float>(old_w + w, static_cast<float>(opt_.max_weight));
        b.sdf[idx] = (b.sdf[idx] * old_w + sdf * w) / (old_w + w);
        b.weight[idx] = new_w;
        touched_.insert(key);
    }

    void TsdfVolume::Integrate(const std::vector<Vec3>& points,
                               const std::vector<Vec3>& normals,
                               const Vec3& origin, double weight, bool orient_to_origin)
    {
        // 点云 → TSDF：对每个点，在其「截断邻域」内的所有体素按「到切平面的有向距离」
        // 更新（splatting）。这样相邻射线之间的体素也会被填上，避免出现三角孔洞。
        // 法线朝传感器定向（PCL 法线无向）。
        const double vs = opt_.voxel_size;
        const double tr = trunc_;
        const double w = weight;
        const size_t n = std::min(points.size(), normals.size());
        for (size_t i = 0; i < n; ++i) {
            const Vec3& p = points[i];
            Vec3 nrm = normals[i];
            if (nrm.norm() < 1e-9) continue;
            nrm.normalize();
            if (orient_to_origin && nrm.dot(origin - p) < 0.0) nrm = -nrm;   // 朝外（自由空间为正）

            const int x0 = static_cast<int>(std::floor((p.x() - tr) / vs));
            const int x1 = static_cast<int>(std::floor((p.x() + tr) / vs));
            const int y0 = static_cast<int>(std::floor((p.y() - tr) / vs));
            const int y1 = static_cast<int>(std::floor((p.y() + tr) / vs));
            const int z0 = static_cast<int>(std::floor((p.z() - tr) / vs));
            const int z1 = static_cast<int>(std::floor((p.z() + tr) / vs));
            for (int vz = z0; vz <= z1; ++vz) {
                for (int vy = y0; vy <= y1; ++vy) {
                    for (int vx = x0; vx <= x1; ++vx) {
                        const Vec3 c((vx + 0.5) * vs, (vy + 0.5) * vs, (vz + 0.5) * vs);
                        const float sdf = static_cast<float>((c - p).dot(nrm));
                        if (std::abs(sdf) > tr) continue;
                        UpdateVoxel(c, sdf, static_cast<float>(w));
                    }
                }
            }
        }
        // 帧末统一标脏：被更新的块及其邻域（保证块边界重网格）
        for (const auto& key : touched_) MarkDirtyAround(key);
        touched_.clear();
    }

    bool TsdfVolume::SampleVoxel(int vx, int vy, int vz, float& sdf) const
    {
        int lx, ly, lz;
        const BlockKey key = BlockOfVoxel(vx, vy, vz, lx, ly, lz);
        auto it = blocks_.find(key);
        if (it == blocks_.end()) return false;
        const int idx = (lz * bs_ + ly) * bs_ + lx;
        if (it->second.weight[idx] <= 0.0f) return false;
        sdf = it->second.sdf[idx];
        return true;
    }

    bool TsdfVolume::SampleTrilinear(const Vec3& pos, float& sdf) const
    {
        const double vs = opt_.voxel_size;
        const double gx = pos.x() / vs - 0.5;
        const double gy = pos.y() / vs - 0.5;
        const double gz = pos.z() / vs - 0.5;
        const int x0 = static_cast<int>(std::floor(gx));
        const int y0 = static_cast<int>(std::floor(gy));
        const int z0 = static_cast<int>(std::floor(gz));
        const double tx = gx - x0, ty = gy - y0, tz = gz - z0;

        float c[8];
        for (int i = 0; i < 8; ++i) {
            const int dx = (i >> 0) & 1, dy = (i >> 1) & 1, dz = (i >> 2) & 1;
            if (!SampleVoxel(x0 + dx, y0 + dy, z0 + dz, c[i])) return false;
        }
        const double c00 = c[0] * (1 - tx) + c[1] * tx;
        const double c10 = c[2] * (1 - tx) + c[3] * tx;
        const double c01 = c[4] * (1 - tx) + c[5] * tx;
        const double c11 = c[6] * (1 - tx) + c[7] * tx;
        const double c0 = c00 * (1 - ty) + c10 * ty;
        const double c1 = c01 * (1 - ty) + c11 * ty;
        sdf = static_cast<float>(c0 * (1 - tz) + c1 * tz);
        return true;
    }

    Vec3 TsdfVolume::GradNormal(const Vec3& pos) const
    {
        const double h = opt_.voxel_size;
        float a, b;
        Vec3 g;
        if (!SampleTrilinear(pos + Vec3(h, 0, 0), a) ||
            !SampleTrilinear(pos - Vec3(h, 0, 0), b)) return Vec3::Zero();
        g.x() = a - b;
        if (!SampleTrilinear(pos + Vec3(0, h, 0), a) ||
            !SampleTrilinear(pos - Vec3(0, h, 0), b)) return Vec3::Zero();
        g.y() = a - b;
        if (!SampleTrilinear(pos + Vec3(0, 0, h), a) ||
            !SampleTrilinear(pos - Vec3(0, 0, h), b)) return Vec3::Zero();
        g.z() = a - b;
        if (g.norm() < 1e-9) return Vec3::Zero();
        return g.normalized();   // sdf 向外为正 → 梯度朝外
    }

    void TsdfVolume::PolygonizeBlock(const BlockKey& key, std::vector<float>& out_v,
                                     std::vector<float>& out_n) const
    {
        const int base_x = key.x * bs_;
        const int base_y = key.y * bs_;
        const int base_z = key.z * bs_;
        const double vs = opt_.voxel_size;

        auto emit = [&](const Vec3& a, const Vec3& b, const Vec3& c) {
            Vec3 face = (b - a).cross(c - a);
            if (face.norm() < 1e-12) return;
            // 用 SDF 梯度（朝外）统一绕序：法线朝内的三角形翻转 → 全网格绕序一致，
            // 开背面剔除也不会漏面。
            Vec3 bb = b, cc = c;
            const Vec3 ref = GradNormal((a + b + c) / 3.0);
            if (ref.norm() > 1e-9 && face.dot(ref) < 0.0) std::swap(bb, cc);
            face = (bb - a).cross(cc - a);
            face.normalize();
            const Vec3 tri[3] = {a, bb, cc};
            for (const auto& p : tri) {
                Vec3 n = GradNormal(p);          // 逐顶点法线（TSDF 梯度）→ 平滑着色
                if (n.norm() < 1e-9) n = face;   // 回退：面法线
                out_v.push_back(static_cast<float>(p.x()));
                out_v.push_back(static_cast<float>(p.y()));
                out_v.push_back(static_cast<float>(p.z()));
                out_n.push_back(static_cast<float>(n.x()));
                out_n.push_back(static_cast<float>(n.y()));
                out_n.push_back(static_cast<float>(n.z()));
            }
        };

        for (int i = 0; i < bs_; ++i) {
            for (int j = 0; j < bs_; ++j) {
                for (int k = 0; k < bs_; ++k) {
                    float val[8];
                    Vec3 pos[8];
                    bool ok = true;
                    for (int c = 0; c < 8; ++c) {
                        const int vx = base_x + i + kCorner[c][0];
                        const int vy = base_y + j + kCorner[c][1];
                        const int vz = base_z + k + kCorner[c][2];
                        if (!SampleVoxel(vx, vy, vz, val[c])) { ok = false; break; }
                        pos[c] = Vec3((vx + 0.5) * vs, (vy + 0.5) * vs, (vz + 0.5) * vs);
                    }
                    if (!ok) continue;

                    float mn = val[0], mx = val[0];
                    for (int c = 1; c < 8; ++c) { mn = std::min(mn, val[c]); mx = std::max(mx, val[c]); }
                    if (mn >= 0.0f || mx <= 0.0f) continue;   // 无过零点

                    for (int t = 0; t < 6; ++t) {
                        const int* tet = kTets[t];
                        float tv[4];
                        Vec3 tp[4];
                        for (int c = 0; c < 4; ++c) { tv[c] = val[tet[c]]; tp[c] = pos[tet[c]]; }
                        int in[4], nin = 0, out[4], nout = 0;
                        for (int c = 0; c < 4; ++c) {
                            if (tv[c] < 0.0f) in[nin++] = c; else out[nout++] = c;
                        }
                        if (nin == 0 || nin == 4) continue;
                        auto interp = [&](int a, int b) {
                            const float ta = tv[a] / (tv[a] - tv[b]);
                            return tp[a] + (tp[b] - tp[a]) * static_cast<double>(ta);
                        };
                        if (nin == 1) {
                            const int a = in[0];
                            emit(interp(a, out[0]), interp(a, out[1]), interp(a, out[2]));
                        } else if (nin == 3) {
                            const int o = out[0];
                            emit(interp(o, in[0]), interp(o, in[2]), interp(o, in[1]));
                        } else {
                            const int a = in[0], b = in[1], c = out[0], d = out[1];
                            const Vec3 p0 = interp(a, c), p1 = interp(a, d);
                            const Vec3 p2 = interp(b, d), p3 = interp(b, c);
                            emit(p0, p1, p2);
                            emit(p0, p2, p3);
                        }
                    }
                }
            }
        }
    }

    void TsdfVolume::UpdateMesh()
    {
        last_deltas_.clear();
        if (dirty_.empty()) return;

        for (const auto& key : dirty_) {
            std::vector<float> v, n;
            PolygonizeBlock(key, v, n);
            const bool had = block_mesh_.count(key) != 0;
            if (v.empty()) {
                block_mesh_.erase(key);
                if (had) {   // 曾经有网格，现在没了 → remove 增量
                    ChunkMesh c;
                    c.chunk_id = PackChunkId(key.x, key.y, key.z);
                    c.origin = ChunkOrigin(key);
                    c.removed = true;
                    c.revision = block_rev_[key] = next_rev_++;
                    last_deltas_.push_back(std::move(c));
                }
            } else {
                block_mesh_[key] = {v, n};
                ChunkMesh c;
                c.chunk_id = PackChunkId(key.x, key.y, key.z);
                c.origin = ChunkOrigin(key);
                c.removed = false;
                c.revision = block_rev_[key] = next_rev_++;
                const Vec3 o = c.origin;
                c.verts = v; c.normals = n;
                for (size_t i = 0; i + 2 < c.verts.size(); i += 3) {
                    c.verts[i]   -= static_cast<float>(o.x());
                    c.verts[i+1] -= static_cast<float>(o.y());
                    c.verts[i+2] -= static_cast<float>(o.z());
                }
                last_deltas_.push_back(std::move(c));
            }
        }
        dirty_.clear();

        mesh_verts_.clear();
        mesh_normals_.clear();
        size_t total = 0;
        for (const auto& kv : block_mesh_) total += kv.second.first.size();
        mesh_verts_.reserve(total);
        mesh_normals_.reserve(total);
        for (const auto& kv : block_mesh_) {
            mesh_verts_.insert(mesh_verts_.end(), kv.second.first.begin(), kv.second.first.end());
            mesh_normals_.insert(mesh_normals_.end(), kv.second.second.begin(), kv.second.second.end());
        }
    }

    std::vector<ChunkMesh> TsdfVolume::DrainChunkDeltas()
    {
        std::vector<ChunkMesh> out = std::move(last_deltas_);
        last_deltas_.clear();
        return out;
    }

    std::vector<ChunkMesh> TsdfVolume::SnapshotChunks() const
    {
        std::vector<ChunkMesh> out;
        out.reserve(block_mesh_.size());
        for (const auto& kv : block_mesh_) {
            if (kv.second.first.empty()) continue;
            ChunkMesh c;
            c.chunk_id = PackChunkId(kv.first.x, kv.first.y, kv.first.z);
            c.origin = ChunkOrigin(kv.first);
            c.removed = false;
            auto it = block_rev_.find(kv.first);
            c.revision = (it != block_rev_.end()) ? it->second : 0;
            const Vec3 o = c.origin;
            c.verts = kv.second.first; c.normals = kv.second.second;
            for (size_t i = 0; i + 2 < c.verts.size(); i += 3) {
                c.verts[i]   -= static_cast<float>(o.x());
                c.verts[i+1] -= static_cast<float>(o.y());
                c.verts[i+2] -= static_cast<float>(o.z());
            }
            out.push_back(std::move(c));
        }
        return out;
    }

    void TsdfVolume::Clear()
    {
        blocks_.clear();
        dirty_.clear();
        touched_.clear();
        block_mesh_.clear();
        mesh_verts_.clear();
        mesh_normals_.clear();
        block_rev_.clear();
        last_deltas_.clear();
        next_rev_ = 1;
    }

}  // namespace RusReconstruction
