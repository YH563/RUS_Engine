// ════════════════════════════════════════════════════════════════════
//  rus_sim_reconstruction 单元测试：surfel 融合（合成几何，纯 CPU）
// ════════════════════════════════════════════════════════════════════

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <random>

#include "rus_sim_reconstruction/sharded_surfel_map.hpp"
#include "rus_sim_reconstruction/surfel_map.hpp"

using namespace RusReconstruction;

namespace {

// 生成 z=0 平面上的规则网格帧（法线 +Z），可加高斯噪声
Frame MakePlaneFrame(int nx, int ny, double step, double noise, uint32_t seed)
{
    Frame f;
    std::mt19937 rng(seed);
    std::normal_distribution<double> nd(0.0, noise);
    f.points.reserve(static_cast<size_t>(nx) * ny);
    for (int i = 0; i < nx; ++i) {
        for (int j = 0; j < ny; ++j) {
            f.points.emplace_back(i * step, j * step, 0.0 + nd(rng));
            f.normals.emplace_back(0.0, 0.0, 1.0);
            f.colors.push_back(0);
        }
    }
    return f;
}

}  // namespace

// ── 重复融合同一平面：面元数不增长（命中匹配）──
TEST(SurfelMap, RepeatFusionKeepsSurfaceCount)
{
    SurfelFusionOptions opt;
    opt.voxel_size = 0.01;
    opt.max_distance = 0.03;
    SurfelMap map(opt);

    const Frame f = MakePlaneFrame(20, 20, 0.01, 0.0, 1);
    map.Fuse(f);
    const size_t after1 = map.SurfaceCount();
    EXPECT_EQ(after1, 400u);           // 20x20 个独立体素
    EXPECT_EQ(map.FramesFused(), 1u);
    EXPECT_EQ(map.PointsFused(), 400u);

    for (int k = 0; k < 4; ++k) map.Fuse(f);
    EXPECT_EQ(map.SurfaceCount(), after1);   // 命中 → 不新增
    EXPECT_EQ(map.FramesFused(), 5u);
    EXPECT_EQ(map.PointsFused(), 400u * 5u);
}

// ── 置信度随观测次数增长 ──
TEST(SurfelMap, ConfidenceAccumulates)
{
    SurfelFusionOptions opt;
    opt.voxel_size = 0.01;
    SurfelMap map(opt);

    Frame f;
    f.points = {Vec3(0.001, 0.001, 0.0)};
    f.normals = {Vec3(0, 0, 1)};
    for (int k = 0; k < 3; ++k) map.Fuse(f);

    ASSERT_EQ(map.SurfaceCount(), 1u);
    const auto snap = map.Snapshot();
    EXPECT_FLOAT_EQ(snap[0].confidence, 3.0f);
}

// ── 多视角带噪观测：位置收敛到真值，法线保持 +Z ──
TEST(SurfelMap, FusesNoisyViewsToTruth)
{
    SurfelFusionOptions opt;
    opt.voxel_size = 0.02;
    opt.max_distance = 0.05;
    SurfelMap map(opt);

    for (uint32_t s = 1; s <= 200; ++s) {
        map.Fuse(MakePlaneFrame(10, 10, 0.02, 0.005, s));
    }

    const auto snap = map.Snapshot();
    ASSERT_FALSE(snap.empty());
    double mean_abs_z = 0.0;
    for (const auto& sf : snap) {
        mean_abs_z += std::abs(sf.position.z());
        EXPECT_GT(sf.normal.dot(Vec3::UnitZ()), 0.99);   // 法线仍朝 +Z
    }
    mean_abs_z /= static_cast<double>(snap.size());
    EXPECT_LT(mean_abs_z, 0.002);   // 平均位置贴近真值 z=0
}

// ── 位姿变换被正确应用（点被搬到 base 系）──
TEST(SurfelMap, AppliesSensorToBaseTransform)
{
    SurfelFusionOptions opt;
    opt.voxel_size = 0.01;
    opt.max_distance = 0.03;
    SurfelMap map(opt);

    Frame f = MakePlaneFrame(20, 20, 0.01, 0.0, 1);
    f.T_base_sensor = Mat4::Identity();
    f.T_base_sensor(0, 3) = 0.5;   // 平移 +0.5 m
    map.Fuse(f);

    double min_x = 1e9, max_x = -1e9;
    for (const auto& sf : map.Snapshot()) {
        min_x = std::min(min_x, sf.position.x());
        max_x = std::max(max_x, sf.position.x());
    }
    EXPECT_GE(min_x, 0.49);
    EXPECT_LE(max_x, 0.70);
}

// ── 同体素内朝向不同的表面不合并 ──
TEST(SurfelMap, IncompatibleNormalsNotMerged)
{
    SurfelFusionOptions opt;
    opt.voxel_size = 0.01;
    opt.max_normal_angle_deg = 30.0;
    SurfelMap map(opt);

    Frame f;
    f.points = {Vec3(0, 0, 0), Vec3(0, 0, 0)};   // 同位置
    f.normals = {Vec3(0, 0, 1), Vec3(1, 0, 0)};  // 正交法线
    f.colors = {0, 0};
    map.Fuse(f);

    EXPECT_EQ(map.SurfaceCount(), 2u);   // 夹角 > 30° → 两个面元
}

// ── 面元上限：达到上限只更新不新增 ──
TEST(SurfelMap, RespectsMaxSurfels)
{
    SurfelFusionOptions opt;
    opt.voxel_size = 0.01;
    SurfelMap map(opt);
    map.SetMaxSurfels(1);

    map.Fuse(MakePlaneFrame(10, 10, 0.01, 0.0, 1));
    EXPECT_EQ(map.SurfaceCount(), 1u);
}

// ── 空帧 / Clear ──
TEST(SurfelMap, EmptyFrameAndClear)
{
    SurfelMap map;
    map.Fuse(Frame{});                 // 空帧：无副作用
    EXPECT_EQ(map.FramesFused(), 0u);

    map.Fuse(MakePlaneFrame(5, 5, 0.01, 0.0, 1));
    EXPECT_GT(map.SurfaceCount(), 0u);
    map.Clear();
    EXPECT_EQ(map.SurfaceCount(), 0u);
    EXPECT_EQ(map.VoxelCount(), 0u);
    EXPECT_EQ(map.PointsFused(), 0u);
    EXPECT_EQ(map.FramesFused(), 0u);
}

// ── 分片并行地图：能融合、法线正确、重复融合稳定 ──
TEST(ShardedSurfelMap, FusesPlaneStably)
{
    SurfelFusionOptions opt;
    opt.voxel_size = 0.01;
    ShardedSurfelMap map(opt, 4, 2);

    const Frame f = MakePlaneFrame(20, 20, 0.01, 0.0, 1);
    map.Fuse(f);
    const size_t after1 = map.SurfaceCount();
    EXPECT_GT(after1, 0u);

    map.Fuse(f);
    EXPECT_EQ(map.SurfaceCount(), after1);   // 命中已建面元 → 不增长

    const auto snap = map.Snapshot();
    ASSERT_FALSE(snap.empty());
    for (const auto& s : snap) {
        EXPECT_GT(s.normal.dot(Vec3::UnitZ()), 0.99);
    }
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
