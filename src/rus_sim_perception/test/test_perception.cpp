// ════════════════════════════════════════════════════════════════════
//  rus_sim_perception 单元测试
//  覆盖纯算法模块（无 ROS 运行时）：
//    - components/pose_interpolator   位姿时间插值
//    - components/sensor_encoder      点云编码 / 解码回环
//    - pointcloud/spatial_transformer 相机→base 变换
//    - pointcloud/map_manager         累积地图 / 上限体素化
//    - pointcloud/cloud_filter_pipeline 滤波链（直通 / 体素 / NaN）
// ════════════════════════════════════════════════════════════════════

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "components/pose_interpolator.hpp"
#include "components/sensor_encoder.hpp"
#include "components/types.hpp"
#include "pointcloud/cloud_filter_pipeline.hpp"
#include "pointcloud/cloud_resampler.hpp"
#include "pointcloud/map_manager.hpp"
#include "pointcloud/spatial_transformer.hpp"

using namespace RusPerception;

namespace {
    pcl::PointXYZRGB MakePoint(float x, float y, float z, uint8_t r = 10, uint8_t g = 20, uint8_t b = 30)
    {
        pcl::PointXYZRGB p;
        p.x = x; p.y = y; p.z = z;
        p.r = r; p.g = g; p.b = b;
        return p;
    }

    Eigen::Isometry3d MakeIso(double x, double y, double z)
    {
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        T.translation() = Eigen::Vector3d(x, y, z);
        return T;
    }
}  // namespace

// ────────────────────────────────────────────────────────────────
//  PoseInterpolator
// ────────────────────────────────────────────────────────────────
TEST(PoseInterpolator, InterpolatesBetweenTwoPoses)
{
    PoseInterpolator interp(16);
    interp.Add(0.0, MakeIso(0.0, 0.0, 0.0));
    interp.Add(1.0, MakeIso(2.0, 0.0, 0.0));

    Eigen::Isometry3d out;
    double diff = -1.0;
    ASSERT_TRUE(interp.Sample(0.5, out, &diff));
    EXPECT_NEAR(out.translation().x(), 1.0, 1e-6);
    EXPECT_NEAR(diff, 0.5, 1e-9);
}

TEST(PoseInterpolator, OutOfRangeReturnsFalse)
{
    PoseInterpolator interp(16);
    interp.Add(0.0, Eigen::Isometry3d::Identity());
    interp.Add(1.0, Eigen::Isometry3d::Identity());

    Eigen::Isometry3d out;
    EXPECT_FALSE(interp.Sample(-0.1, out));   // 早于最早
    EXPECT_FALSE(interp.Sample(1.1, out));    // 晚于最新
}

TEST(PoseInterpolator, IgnoresNonMonotonicAndDuplicate)
{
    PoseInterpolator interp(16);
    EXPECT_TRUE(interp.Add(1.0, Eigen::Isometry3d::Identity()));
    EXPECT_FALSE(interp.Add(0.5, Eigen::Isometry3d::Identity()));  // 乱序
    EXPECT_FALSE(interp.Add(1.0, Eigen::Isometry3d::Identity()));  // 重复
    EXPECT_EQ(interp.Size(), 1u);
}

TEST(PoseInterpolator, LatestPoseAndCacheBound)
{
    PoseInterpolator interp(2);  // 只留 2 帧
    interp.Add(0.0, MakeIso(0.0, 0.0, 0.0));
    interp.Add(1.0, MakeIso(1.0, 0.0, 0.0));
    interp.Add(2.0, MakeIso(2.0, 0.0, 0.0));
    EXPECT_LE(interp.Size(), 2u);

    Eigen::Isometry3d out;
    ASSERT_TRUE(interp.LatestPose(out));
    EXPECT_NEAR(out.translation().x(), 2.0, 1e-6);
}

// ────────────────────────────────────────────────────────────────
//  SensorEncoder：编码 → 解码回环
// ────────────────────────────────────────────────────────────────
TEST(SensorEncoder, PointCloudRoundTrip)
{
    CloudRGB cloud;
    cloud.push_back(MakePoint(0.0f, 0.0f, 0.0f, 255, 0, 0));
    cloud.push_back(MakePoint(0.1f, -0.2f, 0.3f, 0, 255, 0));
    cloud.push_back(MakePoint(-0.4f, 0.5f, -0.05f, 0, 0, 255));

    SensorEncoder enc;
    EncodedFrame out;
    ASSERT_TRUE(enc.EncodePointCloud(cloud, out));
    EXPECT_EQ(out.type, "pointcloud");
    EXPECT_EQ(out.encoding, "zstd");
    EXPECT_EQ(out.points, 3u);
    ASSERT_EQ(out.fields.size(), 4u);

    CloudRGB back;
    ASSERT_TRUE(SensorEncoder::DecodePointCloud(out, back));
    ASSERT_EQ(back.size(), 3u);

    // 反量化误差 ≤ 步长/2（包围盒/65535）；颜色应精确还原
    for (size_t i = 0; i < cloud.size(); ++i) {
        EXPECT_NEAR(back[i].x, cloud[i].x, 1e-4);
        EXPECT_NEAR(back[i].y, cloud[i].y, 1e-4);
        EXPECT_NEAR(back[i].z, cloud[i].z, 1e-4);
        EXPECT_EQ(back[i].r, cloud[i].r);
        EXPECT_EQ(back[i].g, cloud[i].g);
        EXPECT_EQ(back[i].b, cloud[i].b);
    }
}

TEST(SensorEncoder, EmptyCloudFails)
{
    CloudRGB empty;
    SensorEncoder enc;
    EncodedFrame out;
    EXPECT_FALSE(enc.EncodePointCloud(empty, out));
}

// ────────────────────────────────────────────────────────────────
//  SpatialTransformer
// ────────────────────────────────────────────────────────────────
TEST(SpatialTransformer, IdentityPassThrough)
{
    PointCloud::SpatialTransformer tf;  // 默认 camera_to_flange = I
    auto in = CloudRGBPtr(new CloudRGB());
    in->push_back(MakePoint(1.0f, 2.0f, 3.0f));

    CloudRGBPtr out;
    ASSERT_TRUE(tf.Transform(in, Eigen::Matrix4f::Identity(), out));
    ASSERT_EQ(out->size(), 1u);
    EXPECT_NEAR((*out)[0].x, 1.0f, 1e-5);
    EXPECT_NEAR((*out)[0].y, 2.0f, 1e-5);
    EXPECT_NEAR((*out)[0].z, 3.0f, 1e-5);
}

TEST(SpatialTransformer, AppliesTranslation)
{
    PointCloud::SpatialTransformer tf;
    auto in = CloudRGBPtr(new CloudRGB());
    in->push_back(MakePoint(0.0f, 0.0f, 0.0f));

    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    T(0, 3) = 1.5f;   // base←flange 平移 +x
    CloudRGBPtr out;
    ASSERT_TRUE(tf.Transform(in, T, out));
    ASSERT_EQ(out->size(), 1u);
    EXPECT_NEAR((*out)[0].x, 1.5f, 1e-5);
}

// ────────────────────────────────────────────────────────────────
//  MapManager
// ────────────────────────────────────────────────────────────────
TEST(MapManager, AccumulatesFramesAndSnapshot)
{
    PointCloud::MapManager map;
    for (int f = 0; f < 3; ++f) {
        auto frame = CloudRGBPtr(new CloudRGB());
        for (int i = 0; i < 10; ++i) frame->push_back(MakePoint(0.1f * i, 0.0f, 0.0f));
        map.AddFrame(frame);
    }
    EXPECT_EQ(map.FrameCount(), 3u);
    EXPECT_EQ(map.PointCount(), 30u);
    EXPECT_EQ(map.Snapshot()->size(), 30u);

    map.Clear();
    EXPECT_EQ(map.PointCount(), 0u);
    EXPECT_EQ(map.FrameCount(), 0u);
}

TEST(MapManager, MaxPointsTriggersVoxelize)
{
    PointCloud::MapManager map;
    map.SetVoxelLeafSize(0.01f);
    map.SetMaxPoints(200);

    auto frame = CloudRGBPtr(new CloudRGB());
    for (int i = 0; i < 40; ++i)          // 40x40 = 1600 点，1mm 间距，平面 z=0
        for (int j = 0; j < 40; ++j)
            frame->push_back(MakePoint(0.001f * i, 0.001f * j, 0.0f));
    map.AddFrame(frame);

    // 超过上限 → 触发 1cm 体素化：点数应远小于 1600 且 > 0
    EXPECT_GT(map.PointCount(), 0u);
    EXPECT_LT(map.PointCount(), 200u);
}

// ────────────────────────────────────────────────────────────────
//  CloudFilterPipeline
// ────────────────────────────────────────────────────────────────
TEST(CloudFilterPipeline, PassthroughRemovesOutsideRoi)
{
    CloudRGB cloud;
    for (int i = -10; i <= 10; ++i) cloud.push_back(MakePoint(0.0f, 0.0f, 0.1f * i));

    PointCloud::FilterParameter p;
    p.enable_passthrough = true;
    p.passthrough_field = "z";
    p.passthrough_limit_min = -0.5f;
    p.passthrough_limit_max = 0.5f;
    p.enable_statistical = false;
    p.enable_voxel = false;

    PointCloud::CloudFilterPipeline pipe;
    pipe.SetParameter(p);
    std::vector<PointCloud::FilterStageStat> stats;
    ASSERT_TRUE(pipe.Apply(cloud, &stats));
    EXPECT_FALSE(cloud.empty());
    for (const auto& q : cloud) EXPECT_LE(std::abs(q.z), 0.5f + 1e-4f);
    EXPECT_FALSE(stats.empty());   // 直通阶段有统计
}

TEST(CloudFilterPipeline, VoxelReducesDensity)
{
    CloudRGB cloud;
    for (int i = 0; i < 30; ++i)   // 30x30 点，0.5mm 间距（0.015m 面）
        for (int j = 0; j < 30; ++j)
            cloud.push_back(MakePoint(0.0005f * i, 0.0005f * j, 0.0f));

    const size_t before = cloud.size();
    PointCloud::FilterParameter p;
    p.enable_passthrough = false;
    p.enable_statistical = false;
    p.enable_voxel = true;
    p.voxel_leaf_size = 0.005f;

    PointCloud::CloudFilterPipeline pipe;
    pipe.SetParameter(p);
    ASSERT_TRUE(pipe.Apply(cloud));
    EXPECT_GT(cloud.size(), 0u);
    EXPECT_LT(cloud.size(), before);
}

TEST(CloudFilterPipeline, RemovesNaN)
{
    CloudRGB cloud;
    cloud.push_back(MakePoint(0.0f, 0.0f, 0.0f));
    pcl::PointXYZRGB bad;
    bad.x = std::numeric_limits<float>::quiet_NaN();
    bad.y = 0.0f; bad.z = 0.0f;
    cloud.push_back(bad);

    PointCloud::FilterParameter p;
    p.enable_passthrough = false;
    p.enable_statistical = false;
    p.enable_voxel = false;

    PointCloud::CloudFilterPipeline pipe;
    pipe.SetParameter(p);
    ASSERT_TRUE(pipe.Apply(cloud));
    EXPECT_EQ(cloud.size(), 1u);   // NaN 被清除
}

TEST(CloudFilterPipeline, ResampleStageUniformizesSpacing)
{
    // 稠密平面（0.4mm 间距）→ 开 resample(目标 1cm)：点数下降、间距落在目标附近
    CloudRGB cloud;
    for (int i = 0; i < 80; ++i)
        for (int j = 0; j < 80; ++j)
            cloud.push_back(MakePoint(0.0004f * i, 0.0004f * j, 0.0f));
    const size_t before = cloud.size();

    PointCloud::FilterParameter p;
    p.enable_passthrough = false;
    p.enable_statistical = false;
    p.enable_voxel = false;
    p.enable_resample = true;
    p.resample_target_spacing = 0.01f;
    p.resample_mls_radius = 0.03f;

    PointCloud::CloudFilterPipeline pipe;
    pipe.SetParameter(p);
    std::vector<PointCloud::FilterStageStat> stats;
    ASSERT_TRUE(pipe.Apply(cloud, &stats));

    EXPECT_GT(cloud.size(), 0u);
    EXPECT_LT(cloud.size(), before);

    bool has_resample = false;
    for (const auto& st : stats) if (st.stage == "resample") has_resample = true;
    EXPECT_TRUE(has_resample);

    const PointCloud::SpacingStats s = PointCloud::ComputeSpacingStats(cloud);
    // 贪心均匀采样保证间距 ≥ target；MLS 平滑后允许略小
    EXPECT_GT(s.median, 0.5f * p.resample_target_spacing);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
