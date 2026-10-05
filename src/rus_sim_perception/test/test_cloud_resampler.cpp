// 点云均匀重采样 单测：非均匀平面 → 间距均匀化 + 法线；球面法线朝外。
#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "pointcloud/cloud_resampler.hpp"

using namespace RusPerception;              // CloudRGB
using namespace RusPerception::PointCloud;  // Resample / ResampleOptions / ...



// 非均匀密度平面（中心密、边缘疏）→ 重采样后间距集中在目标附近
TEST(CloudResampler, UniformizesSpacing)
{
    const float target = 0.01f;
    CloudRGB cloud;
    std::mt19937 rng(1);
    std::uniform_real_distribution<float> u(-1.0f, 1.0f);
    std::normal_distribution<float> nz(0.0f, 0.0004f);
    for (int i = 0; i < 30000; ++i) {
        const float s = u(rng);
        const float x = 0.15f * s * std::abs(s);   // 平方 → 中心密
        const float y = 0.15f * u(rng);
        pcl::PointXYZRGB p;
        p.x = x; p.y = y; p.z = nz(rng);
        p.r = p.g = p.b = 200;
        cloud.push_back(p);
    }

    ResampleOptions opt;
    opt.voxel_leaf = target;
    opt.mls_target_spacing = target;
    opt.mls_search_radius = target * 3.0f;

    ResampleResult r;
    Resample(cloud, r, opt);
    ASSERT_GT(r.cloud.size(), 100u);
    ASSERT_EQ(r.normals.size(), r.cloud.size());

    const SpacingStats s = ComputeSpacingStats(r.cloud);
    EXPECT_GT(s.median, 0.5f * target);
    EXPECT_LT(s.median, 1.8f * target);
    EXPECT_LT(s.p95, 3.0f * target);   // 尾部不再像原始那样差异巨大

    // 平面法线应接近 +Z（MLS 法线无向，取 |dot|）
    size_t good = 0;
    for (const auto& n : r.normals) if (std::abs(n.z()) > 0.9f) ++good;
    EXPECT_GT(good, r.normals.size() * 9 / 10);
}

// 球面重采样：法线沿径向
TEST(CloudResampler, SphereNormalsRadial)
{
    CloudRGB cloud;
    std::mt19937 rng(2);
    std::uniform_real_distribution<float> u(-1.0f, 1.0f);
    const float R = 0.1f;
    while (cloud.size() < 20000) {
        const float x = u(rng), y = u(rng), z = u(rng);
        const float r2 = x * x + y * y + z * z;
        if (r2 > 1.0f || r2 < 1e-3f) continue;
        const float inv = R / std::sqrt(r2);
        pcl::PointXYZRGB p;
        p.x = x * inv; p.y = y * inv; p.z = z * inv;
        p.r = p.g = p.b = 120;
        cloud.push_back(p);
    }

    ResampleOptions opt;
    opt.voxel_leaf = 0.006f;
    opt.mls_target_spacing = 0.005f;
    opt.mls_search_radius = 0.015f;

    ResampleResult r;
    Resample(cloud, r, opt);
    ASSERT_GT(r.cloud.size(), 100u);

    size_t good = 0;
    for (size_t i = 0; i < r.cloud.size(); ++i) {
        Eigen::Vector3f p(r.cloud[i].x, r.cloud[i].y, r.cloud[i].z);
        if (p.norm() < 1e-6f) continue;
        if (std::abs(r.normals[i].dot(p.normalized())) > 0.9f) ++good;
    }
    EXPECT_GT(good, r.cloud.size() * 8 / 10);
}

// 间距统计基本行为
TEST(CloudResampler, SpacingStatsBasic)
{
    CloudRGB grid;
    for (int i = 0; i < 10; ++i)
        for (int j = 0; j < 10; ++j) {
            pcl::PointXYZRGB p;
            p.x = i * 0.01f; p.y = j * 0.01f; p.z = 0;
            grid.push_back(p);
        }
    const SpacingStats s = ComputeSpacingStats(grid);
    EXPECT_EQ(s.n, 100u);
    EXPECT_NEAR(s.median, 0.01f, 0.002f);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
