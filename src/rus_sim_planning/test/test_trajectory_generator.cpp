// ════════════════════════════════════════════════════════════════════
//  rus_sim_planning 单元测试：点云轨迹生成器
//  合成平面点云 + 两端点 → 生成贴合轨迹；验证输出非空、点有效、贴面。
//  （不做碰撞检查，与当前阶段一致）
// ════════════════════════════════════════════════════════════════════

#include <gtest/gtest.h>

#include <cmath>

#include <geometry_msgs/msg/pose.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "rus_sim_planning/trajectory_generator.hpp"

using namespace RusSimPlanning;

TEST(TrajectoryGenerator, GeneratesSurfacePathOnPlane)
{
    auto cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    const int N = 41;
    const double half = 0.1;
    for (int i = 0; i < N; ++i)
        for (int j = 0; j < N; ++j) {
            pcl::PointXYZ p;
            p.x = static_cast<float>(-half + 2.0 * half * i / (N - 1));
            p.y = static_cast<float>(-half + 2.0 * half * j / (N - 1));
            p.z = 0.0f;
            cloud->push_back(p);
        }

    TrajectoryGenerator gen;
    ASSERT_TRUE(gen.LoadCloud(cloud)) << "LoadCloud 失败（点云为空或法线估计失败）";
    EXPECT_TRUE(gen.IsInitialized());

    TrajectoryParameter param;
    param.graph_k = 10;
    param.normal_k = 10;
    param.projection_k = 10;
    gen.SetParameter(param);

    geometry_msgs::msg::Pose start;
    start.position.x = -0.08; start.position.y = 0.0; start.position.z = 0.0;
    start.orientation.w = 1.0;
    geometry_msgs::msg::Pose goal;
    goal.position.x = 0.08; goal.position.y = 0.0; goal.position.z = 0.0;
    goal.orientation.w = 1.0;

    ASSERT_TRUE(gen.GenerateTrajectory(start, goal));
    auto traj = gen.GetTrajectory();
    ASSERT_TRUE(traj.has_value());
    const auto& path = traj->get();
    ASSERT_GT(path.size(), 1u);

    for (const auto& p : path) {
        EXPECT_TRUE(std::isfinite(p.position.x));
        EXPECT_TRUE(std::isfinite(p.position.y));
        EXPECT_TRUE(std::isfinite(p.position.z));
        EXPECT_LE(std::abs(p.position.z), 0.05);   // 贴合 z=0 平面
        EXPECT_LE(std::abs(p.position.x), 0.15);
        EXPECT_LE(std::abs(p.position.y), 0.15);
    }
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
