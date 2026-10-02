// ════════════════════════════════════════════════════════════════════
//  rus_sim_planning 单元测试：轨迹插值（TrajectoryInterpolator）
//  纯算法，不依赖 ROS 运行时。
// ════════════════════════════════════════════════════════════════════

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "rus_sim_planning/trajectory_interpolator.hpp"

using RusSimPlanning::TrajectoryInterpolator;
using Trajectory = RusSimPlanning::Trajectory;
using geometry_msgs::msg::Pose;

namespace {

// 造一个位姿：位置 (x,y,z)，绕 z 轴 yaw（默认 0）
Pose PoseAt(double x, double y, double z, double yaw = 0.0)
{
    Pose p;
    p.position.x = x;
    p.position.y = y;
    p.position.z = z;
    p.orientation.w = std::cos(yaw / 2.0);
    p.orientation.z = std::sin(yaw / 2.0);
    return p;
}

}  // namespace

// ── 异常：不足两个路径点无法插值 ──
TEST(TrajectoryInterpolator, RejectsFewerThanTwoWaypoints)
{
    TrajectoryInterpolator it;

    it.SetWaypoints({});
    EXPECT_FALSE(it.Interpolate(4));
    EXPECT_TRUE(it.DenseTrajectory().empty());

    it.SetWaypoints({PoseAt(0, 0, 0)});
    EXPECT_FALSE(it.Interpolate(4));
    EXPECT_EQ(it.Size(), 1u);  // 原地输出（实现：dense_ = waypoints_）
}

// ── 正常：点数公式 + 首尾端点 ──
TEST(TrajectoryInterpolator, CountAndEndpoints)
{
    TrajectoryInterpolator it;
    it.SetWaypoints({PoseAt(0, 0, 0), PoseAt(1, 0, 0)});
    ASSERT_TRUE(it.Interpolate(4));

    // (2-1)*4 + 1 = 5
    ASSERT_EQ(it.Size(), 5u);
    EXPECT_DOUBLE_EQ(it.DenseTrajectory().front().position.x, 0.0);
    EXPECT_DOUBLE_EQ(it.DenseTrajectory().back().position.x, 1.0);
}

// ── 正常：位置为线性插值 ──
TEST(TrajectoryInterpolator, LinearPositionMidpoint)
{
    TrajectoryInterpolator it;
    it.SetWaypoints({PoseAt(0, 0, 0), PoseAt(2, 4, 6)});
    ASSERT_TRUE(it.Interpolate(2));  // 采样 t = 0, 0.5，再补末尾

    const auto& d = it.DenseTrajectory();
    ASSERT_EQ(d.size(), 3u);
    EXPECT_DOUBLE_EQ(d[1].position.x, 1.0);
    EXPECT_DOUBLE_EQ(d[1].position.y, 2.0);
    EXPECT_DOUBLE_EQ(d[1].position.z, 3.0);
    EXPECT_DOUBLE_EQ(d[2].position.x, 2.0);
}

// ── 正常：姿态为 slerp（0° → 180°，中点应为 90°）──
TEST(TrajectoryInterpolator, SlerpOrientationMidpoint)
{
    TrajectoryInterpolator it;
    it.SetWaypoints({PoseAt(0, 0, 0, 0.0), PoseAt(0, 0, 0, M_PI)});
    ASSERT_TRUE(it.Interpolate(2));

    const auto& d = it.DenseTrajectory();
    ASSERT_EQ(d.size(), 3u);
    const auto& q = d[1].orientation;
    EXPECT_NEAR(q.w, std::cos(M_PI / 4.0), 1e-9);
    EXPECT_NEAR(q.z, std::sin(M_PI / 4.0), 1e-9);
    EXPECT_NEAR(q.x, 0.0, 1e-9);
    EXPECT_NEAR(q.y, 0.0, 1e-9);
}

// ── 边界：points_per_segment < 1 钳为 1 ──
TEST(TrajectoryInterpolator, ClampsPointsPerSegmentToOne)
{
    TrajectoryInterpolator it;
    it.SetWaypoints({PoseAt(0, 0, 0), PoseAt(1, 0, 0), PoseAt(2, 0, 0)});
    ASSERT_TRUE(it.Interpolate(0));

    ASSERT_EQ(it.Size(), 3u);  // 每段 1 点 + 末尾 = 3
    EXPECT_DOUBLE_EQ(it.DenseTrajectory()[0].position.x, 0.0);
    EXPECT_DOUBLE_EQ(it.DenseTrajectory()[1].position.x, 1.0);
    EXPECT_DOUBLE_EQ(it.DenseTrajectory()[2].position.x, 2.0);
}

// ── 推进：NextTarget 逐个输出，走完 IsFinished ──
TEST(TrajectoryInterpolator, NextTargetWalkAndFinish)
{
    TrajectoryInterpolator it;
    it.SetWaypoints({PoseAt(0, 0, 0), PoseAt(1, 0, 0)});
    ASSERT_TRUE(it.Interpolate(2));  // 3 点：0.0, 0.5, 1.0
    EXPECT_FALSE(it.IsFinished());

    auto t0 = it.NextTarget();
    ASSERT_TRUE(t0.has_value());
    EXPECT_DOUBLE_EQ(t0->position.x, 0.0);
    EXPECT_EQ(it.CurrentIndex(), 1u);

    auto t1 = it.NextTarget();
    ASSERT_TRUE(t1.has_value());
    EXPECT_DOUBLE_EQ(t1->position.x, 0.5);

    auto t2 = it.NextTarget();
    ASSERT_TRUE(t2.has_value());
    EXPECT_DOUBLE_EQ(t2->position.x, 1.0);

    EXPECT_TRUE(it.IsFinished());
    EXPECT_FALSE(it.NextTarget().has_value());
}

// ── CurrentTarget 只读不推进 ──
TEST(TrajectoryInterpolator, CurrentTargetDoesNotAdvance)
{
    TrajectoryInterpolator it;
    it.SetWaypoints({PoseAt(0, 0, 0), PoseAt(1, 0, 0)});
    ASSERT_TRUE(it.Interpolate(2));

    ASSERT_TRUE(it.CurrentTarget().has_value());
    EXPECT_EQ(it.CurrentIndex(), 0u);
    it.NextTarget();
    EXPECT_EQ(it.CurrentIndex(), 1u);
}

// ── Rewind 回到起点；Reset 清空 ──
TEST(TrajectoryInterpolator, RewindAndReset)
{
    TrajectoryInterpolator it;
    it.SetWaypoints({PoseAt(0, 0, 0), PoseAt(1, 0, 0)});
    ASSERT_TRUE(it.Interpolate(2));
    it.NextTarget();
    it.NextTarget();

    it.Rewind();
    EXPECT_EQ(it.CurrentIndex(), 0u);
    EXPECT_FALSE(it.IsFinished());

    it.Reset();
    EXPECT_EQ(it.Size(), 0u);
    EXPECT_TRUE(it.Waypoints().empty());
    EXPECT_TRUE(it.IsFinished());
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
