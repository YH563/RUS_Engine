// ════════════════════════════════════════════════════════════════════
//  rus_sim_driver 单元测试：指令解析（ParseCommand）
//  纯逻辑，无 ROS 运行时；覆盖运动指令 / 驱动控制 / 未知指令回退。
// ════════════════════════════════════════════════════════════════════

#include <gtest/gtest.h>

#include <string>
#include <variant>
#include <vector>

#include "components/types.hpp"

using namespace RusRobotDriver;

TEST(CmdParser, MovejParsesJointMotion)
{
    RobotCommand c = ParseCommand("movej", {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.8, 0.7});
    const auto* m = std::get_if<MotionCommand>(&c);
    ASSERT_NE(m, nullptr);
    EXPECT_EQ(m->type, MOTION_TYPE_JOINT);
    ASSERT_EQ(m->target.size(), 6);
    EXPECT_NEAR(m->target[0], 0.1, 1e-9);
    EXPECT_NEAR(m->speed, 0.8, 1e-9);
    EXPECT_NEAR(m->acceleration, 0.7, 1e-9);
}

TEST(CmdParser, MovelParsesCartesianMotion)
{
    RobotCommand c = ParseCommand("movel", {0.3, -0.1, 0.4, 0.0, 1.57, 0.0});
    const auto* m = std::get_if<MotionCommand>(&c);
    ASSERT_NE(m, nullptr);
    EXPECT_EQ(m->type, MOTION_TYPE_CART);
    ASSERT_EQ(m->target.size(), 6);
    EXPECT_NEAR(m->target[2], 0.4, 1e-9);
    EXPECT_NEAR(m->speed, 0.5, 1e-9);   // 缺省
}

TEST(CmdParser, ServoVariantsParse)
{
    RobotCommand a = ParseCommand("servoj", {0, 0, 0, 0, 0, 0});
    const auto* sj = std::get_if<MotionCommand>(&a);
    ASSERT_NE(sj, nullptr);
    EXPECT_EQ(sj->type, MOTION_TYPE_SERVOJ);

    RobotCommand b = ParseCommand("servo_cart", {0, 0, 0, 0, 0, 0});
    const auto* sc = std::get_if<MotionCommand>(&b);
    ASSERT_NE(sc, nullptr);
    EXPECT_EQ(sc->type, MOTION_TYPE_SERVOC);
}

TEST(CmdParser, StartJogAxisDirAndSpeed)
{
    // args = [ref, nb, dir, vel%, acc%]；ref=0 → JOG_0（关节点动）
    RobotCommand a = ParseCommand("start_jog", {0, 2, 1, 50, 40});
    const auto* m = std::get_if<MotionCommand>(&a);
    ASSERT_NE(m, nullptr);
    EXPECT_EQ(m->type, MOTION_TYPE_JOG_0);
    EXPECT_EQ(m->jog_axis, 2);
    EXPECT_EQ(m->jog_dir, 1);
    EXPECT_NEAR(m->speed, 0.5, 1e-9);          // 50% → 0.5
    EXPECT_NEAR(m->acceleration, 0.4, 1e-9);   // 40% → 0.4

    // ref=2 → JOG_1（基坐标系点动）；ref=4 → JOG_2（工具坐标系点动）
    RobotCommand b = ParseCommand("start_jog", {2, 1, 0, 100, 100});
    const auto* mb = std::get_if<MotionCommand>(&b);
    ASSERT_NE(mb, nullptr);
    EXPECT_EQ(mb->type, MOTION_TYPE_JOG_1);

    RobotCommand c = ParseCommand("start_jog", {4, 3, 0, 100, 100});
    const auto* mt = std::get_if<MotionCommand>(&c);
    ASSERT_NE(mt, nullptr);
    EXPECT_EQ(mt->type, MOTION_TYPE_JOG_2);
}

TEST(CmdParser, NonMotionCommands)
{
    RobotCommand a = ParseCommand("connect", {});
    EXPECT_NE(std::get_if<ConnectCmd>(&a), nullptr);

    RobotCommand b = ParseCommand("is_motion_done", {});
    EXPECT_NE(std::get_if<IsMotionDoneCmd>(&b), nullptr);

    RobotCommand c = ParseCommand("get_driver_type", {});
    EXPECT_NE(std::get_if<GetDriverTypeCmd>(&c), nullptr);

    RobotCommand d = ParseCommand("reset", {0, 1});
    const auto* r = std::get_if<ResetCmd>(&d);
    ASSERT_NE(r, nullptr);
    EXPECT_EQ(r->mode, 0);
    EXPECT_EQ(r->enable, 1);
}

TEST(CmdParser, UnknownFallsBackToSafeStop)
{
    // 未知指令返回安全指令 StopCmd（不抛异常、不误动）
    RobotCommand c = ParseCommand("nonsense_cmd", {});
    EXPECT_NE(std::get_if<StopCmd>(&c), nullptr);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
