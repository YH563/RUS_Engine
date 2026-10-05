// ════════════════════════════════════════════════════════════════════
//  rus_sim_bridge 单元测试：指令 → 模块注册表（纯路由域，header-only）
//  覆盖注册 / 查询 / 本地指令 / 运行时改目标 / 模块名辅助。
// ════════════════════════════════════════════════════════════════════

#include <gtest/gtest.h>

#include <string_view>

#include "rus_sim_utils/command_registry.hpp"

using namespace RusUtils;

TEST(CommandRegistry, RegisterAndQuery)
{
    CommandRegistry reg;
    reg.Register("plan", {Module::PLANNING});

    EXPECT_TRUE(reg.IsRegistered("plan"));
    EXPECT_FALSE(reg.IsLocal("plan"));
    const auto targets = reg.TargetsOf("plan");
    ASSERT_EQ(targets.size(), 1u);
    EXPECT_EQ(targets[0], Module::PLANNING);
}

TEST(CommandRegistry, EmptyTargetsAreLocal)
{
    CommandRegistry reg;
    reg.Register("shutdown", {});   // 空列表 = bridge 本地处理
    EXPECT_TRUE(reg.IsRegistered("shutdown"));
    EXPECT_TRUE(reg.IsLocal("shutdown"));
}

TEST(CommandRegistry, UnknownCommandHasNoTargets)
{
    CommandRegistry reg;
    EXPECT_FALSE(reg.IsRegistered("nope"));
    EXPECT_TRUE(reg.TargetsOf("nope").empty());
}

TEST(CommandRegistry, SetTargetsUpdatesFanout)
{
    CommandRegistry reg;
    reg.Register("stop", {Module::PLANNING});
    reg.SetTargets("stop", {Module::PLANNING, Module::DRIVER});
    EXPECT_EQ(reg.TargetsOf("stop").size(), 2u);
    EXPECT_EQ(reg.TargetsOf("stop")[1], Module::DRIVER);
}

TEST(CommandRegistry, ModuleNameHelpers)
{
    EXPECT_STREQ(module_service_name(Module::PLANNING), "/planning/command");
    EXPECT_STREQ(module_service_name(Module::DRIVER), "/driver/command");
    EXPECT_EQ(module_name(Module::RECORDER), std::string_view("recorder"));
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
