// TSDF 体 + 增量 Marching Tetrahedra 网格 单测。
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "rus_sim_reconstruction/tsdf_volume.hpp"

using namespace RusReconstruction;

namespace {
void AddPlane(std::vector<Vec3>& p, std::vector<Vec3>& n, double x0, double x1)
{
    const int m = 40;
    for (int i = 0; i <= m; ++i) {
        for (int j = 0; j <= m; ++j) {
            const double x = x0 + (x1 - x0) * i / m;
            const double y = -0.1 + 0.2 * j / m;
            p.emplace_back(x, y, 0.0);
            n.emplace_back(0, 0, 1);
        }
    }
}
}  // namespace

// 平面 → TSDF 积分 → 网格三角面存在，且都贴近 z=0
TEST(TsdfVolume, PlaneProducesMesh)
{
    TsdfVolume::Options o;
    o.voxel_size = 0.01;
    o.truncation = 0.03;
    TsdfVolume vol(o);

    std::vector<Vec3> p, n;
    AddPlane(p, n, -0.1, 0.1);
    vol.Integrate(p, n, Vec3(0, 0, 0.3));
    vol.UpdateMesh();

    ASSERT_GT(vol.TriangleCount(), 0u);
    const auto& V = vol.MeshVertices();
    for (size_t i = 2; i < V.size(); i += 3) {
        EXPECT_NEAR(V[i], 0.0f, static_cast<float>(2 * o.voxel_size));
    }
    EXPECT_GT(vol.BlockCount(), 0u);
}

// 增量：先积一半 → 再积另一半 → 网格三角面增加
TEST(TsdfVolume, IncrementalGrows)
{
    TsdfVolume::Options o;
    o.voxel_size = 0.01;
    o.truncation = 0.03;
    TsdfVolume vol(o);

    std::vector<Vec3> p1, n1, p2, n2;
    AddPlane(p1, n1, -0.1, 0.0);
    AddPlane(p2, n2, 0.0, 0.1);

    vol.Integrate(p1, n1, Vec3(0, 0, 0.3));
    vol.UpdateMesh();
    const size_t t1 = vol.TriangleCount();
    ASSERT_GT(t1, 0u);

    vol.Integrate(p2, n2, Vec3(0, 0, 0.3));
    EXPECT_TRUE(vol.HasDirty());
    vol.UpdateMesh();
    const size_t t2 = vol.TriangleCount();
    EXPECT_GT(t2, t1);
}

// 无脏块时 UpdateMesh 是空操作；Clear 复位
TEST(TsdfVolume, NoDirtyAndClear)
{
    TsdfVolume::Options o;
    o.voxel_size = 0.01;
    o.truncation = 0.03;
    TsdfVolume vol(o);

    std::vector<Vec3> p, n;
    AddPlane(p, n, -0.1, 0.1);
    vol.Integrate(p, n, Vec3(0, 0, 0.3));
    vol.UpdateMesh();
    EXPECT_FALSE(vol.HasDirty());
    const size_t t = vol.TriangleCount();

    vol.UpdateMesh();                 // 无脏块 → 不变
    EXPECT_EQ(vol.TriangleCount(), t);

    vol.Clear();
    EXPECT_EQ(vol.TriangleCount(), 0u);
    EXPECT_EQ(vol.BlockCount(), 0u);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
