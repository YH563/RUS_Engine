#pragma once

// ════════════════════════════════════════════════════════════════════
//  稀疏体素哈希 TSDF 体（KinectFusion / Voxblox 风格）
//  ────────────────────────────────────────────────────────────────────
//  - 已知位姿：每帧输入「点 + 法向 + 传感器原点」，沿视线做投影式积分
//    （Voxblox pointcloud 积分），更新截断符号距离场 TSDF；
//  - 体素按 8^3 块哈希分配（稀疏，省内存）；
//  - 只对视场覆盖到的块做 Marching Tetrahedra，产出**分块增量**三角网格：
//    Integrate 标记脏块，UpdateMesh 只重算脏块并合并。
//
//  纯 Eigen + std（无 ROS / PCL），可脱离运行时单测。
// ════════════════════════════════════════════════════════════════════

#include <array>
#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "rus_sim_reconstruction/types.hpp"

namespace RusReconstruction {

    class TsdfVolume {
    public:
        struct Options {
            double voxel_size = 0.006;        // 体素边长（米）
            int    block_size = 8;            // 每块体素数（边长）
            double truncation = 0.018;        // 截断距离（米），典型 3×voxel
            double max_weight = 64.0;         // 单个体素权重上限
        };

        TsdfVolume() : TsdfVolume(Options{}) {}
        explicit TsdfVolume(const Options& options);

        /// 积分一帧：点/法向（base 系），origin = 传感器原点（base 系）
        /// @param orient_to_origin true=法线朝传感器翻转（逐帧视角）；false=直接信任输入法线（一次性整套点云）
        void Integrate(const std::vector<Vec3>& points,
                       const std::vector<Vec3>& normals,
                       const Vec3& origin,
                       double weight = 1.0,
                       bool orient_to_origin = true);

        /// 只重建脏块并刷新合并网格（增量）
        void UpdateMesh();

        /// 合并网格（三角汤；每三角形 9 个 float 位置 + 9 个 float 法线）
        const std::vector<float>& MeshVertices() const { return mesh_verts_; }
        const std::vector<float>& MeshNormals() const { return mesh_normals_; }
        size_t TriangleCount() const { return mesh_verts_.size() / 9; }

        size_t BlockCount() const { return blocks_.size(); }
        size_t DirtyBlockCount() const { return dirty_.size(); }
        bool HasDirty() const { return !dirty_.empty(); }

        void Clear();

    private:
        struct BlockKey {
            int32_t x = 0, y = 0, z = 0;
            bool operator==(const BlockKey& o) const { return x == o.x && y == o.y && z == o.z; }
        };
        struct BlockKeyHash {
            size_t operator()(const BlockKey& k) const {
                uint64_t h = 1469598103934665603ull;
                auto mix = [&h](int32_t v) { h ^= static_cast<uint32_t>(v); h *= 1099511628211ull; };
                mix(k.x); mix(k.y); mix(k.z);
                return static_cast<size_t>(h);
            }
        };
        struct Block {
            std::array<float, 512> sdf{};
            std::array<float, 512> weight{};
        };

        static int floor_div(int a, int b);
        BlockKey BlockOfVoxel(int vx, int vy, int vz, int& lx, int& ly, int& lz) const;
        void UpdateVoxel(const Vec3& pos, float sdf, float w);
        void MarkDirtyAround(const BlockKey& k);

        /// 采样全局体素 (vx,vy,vz) 的 sdf；valid=false 表示未知
        bool SampleVoxel(int vx, int vy, int vz, float& sdf) const;

        /// 三线性插值 sdf（用于梯度法线）；任一邻域体素未知则返回 false
        bool SampleTrilinear(const Vec3& pos, float& sdf) const;

        /// 由 TSDF 梯度求表面法线（平滑着色）；失败返回零向量
        Vec3 GradNormal(const Vec3& pos) const;

        /// 对某块做 Marching Tetrahedra，写入 out
        void PolygonizeBlock(const BlockKey& key, std::vector<float>& out_v,
                             std::vector<float>& out_n) const;

        Options opt_;
        int bs_ = 8;
        double trunc_ = 0.018;
        std::unordered_map<BlockKey, Block, BlockKeyHash> blocks_;
        std::unordered_set<BlockKey, BlockKeyHash> dirty_;
        std::unordered_set<BlockKey, BlockKeyHash> touched_;  // 本帧被更新的块
        // 每块的网格（增量：只重算脏块）；合并网格由此拼接
        std::unordered_map<BlockKey, std::pair<std::vector<float>, std::vector<float>>,
                           BlockKeyHash> block_mesh_;
        std::vector<float> mesh_verts_;
        std::vector<float> mesh_normals_;
    };

}  // namespace RusReconstruction
