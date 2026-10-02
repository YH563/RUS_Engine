// ════════════════════════════════════════════════════════════════════
//  rus_sim_utils 单元测试：协议编解码 / 指令解析 / 位姿工具
//  header-only，直接编进本测试可执行。
// ════════════════════════════════════════════════════════════════════

#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include "rus_sim_utils/command_types.hpp"
#include "rus_sim_utils/protocol.hpp"
#include "rus_sim_utils/utils.hpp"

using namespace RusUtils;

// ────────────────────────────────────────────────────────────────
//  protocol：SensorFrame 二进制往返
// ────────────────────────────────────────────────────────────────
TEST(Protocol, SensorFramePointCloudRoundTrip)
{
    SensorFrame f;
    f.type = std::string(SensorType::kPointCloud);
    f.timestamp = 123.456;
    f.seq = 7;
    f.frame_id = "base_link";
    f.encoding = "zstd";
    f.scope = std::string(SensorScope::kMap);
    f.points = 3;
    f.fields = {"x", "y", "z", "rgb"};
    f.dtype = "int16";
    f.range_min = {-1.0, -2.0, -3.0};
    f.range_max = {1.0, 2.0, 3.0};
    f.payload = {1, 2, 3, 4, 5};

    const auto buf = EncodeSensorFrame(f);
    SensorFrame g;
    ASSERT_TRUE(DecodeSensorFrame(buf, g));

    EXPECT_EQ(g.type, f.type);
    EXPECT_DOUBLE_EQ(g.timestamp, f.timestamp);
    EXPECT_EQ(g.seq, f.seq);
    EXPECT_EQ(g.frame_id, f.frame_id);
    EXPECT_EQ(g.encoding, f.encoding);
    EXPECT_EQ(g.scope, f.scope);
    EXPECT_EQ(g.points, f.points);
    EXPECT_EQ(g.fields, f.fields);
    EXPECT_EQ(g.dtype, f.dtype);
    ASSERT_EQ(g.range_min.size(), 3u);
    EXPECT_DOUBLE_EQ(g.range_min[0], -1.0);
    EXPECT_DOUBLE_EQ(g.range_max[2], 3.0);
    EXPECT_EQ(g.payload, f.payload);
}

TEST(Protocol, SensorFrameImageRoundTrip)
{
    SensorFrame f;
    f.type = std::string(SensorType::kImage);
    f.timestamp = 9.5;
    f.seq = 1;
    f.frame_id = "camera";
    f.encoding = "raw";
    f.scope = std::string(SensorScope::kFrame);
    f.width = 640;
    f.height = 480;
    f.image_encoding = "rgb8";
    f.step = 1920;
    f.payload = {0xAA, 0xBB};

    const auto buf = EncodeSensorFrame(f);
    SensorFrame g;
    ASSERT_TRUE(DecodeSensorFrame(buf, g));

    EXPECT_EQ(g.type, std::string(SensorType::kImage));
    EXPECT_EQ(g.width, 640u);
    EXPECT_EQ(g.height, 480u);
    EXPECT_EQ(g.image_encoding, "rgb8");
    EXPECT_EQ(g.step, 1920u);
    EXPECT_EQ(g.payload, f.payload);
}

TEST(Protocol, DecodeSensorFrameRejectsTruncated)
{
    SensorFrame out;
    EXPECT_FALSE(DecodeSensorFrame({}, out));
    EXPECT_FALSE(DecodeSensorFrame({1, 2, 3}, out));       // 不足 4 字节头长度
    EXPECT_FALSE(DecodeSensorFrame({0xFF, 0, 0, 0}, out)); // 声称头很长但无数据
}

TEST(Protocol, ParseCommandMessage)
{
    CommandMessage m;
    ASSERT_TRUE(ParseCommandMessage(
        R"({"id":42,"cmd":"movel","args":[1,2,3],"text":"/tmp/a.rusrec"})", m));
    EXPECT_EQ(m.id, 42u);
    EXPECT_EQ(m.cmd, "movel");
    ASSERT_EQ(m.args.size(), 3u);
    EXPECT_DOUBLE_EQ(m.args[0], 1.0);
    EXPECT_EQ(m.text, "/tmp/a.rusrec");

    // 无 cmd → 解析失败
    CommandMessage bad;
    EXPECT_FALSE(ParseCommandMessage(R"({"id":1,"args":[]})", bad));

    // 无 text 字段 → 空串
    CommandMessage no_text;
    ASSERT_TRUE(ParseCommandMessage(R"({"id":2,"cmd":"plan","args":[]})", no_text));
    EXPECT_TRUE(no_text.text.empty());
}

TEST(Protocol, SerializeResultReplyAndEvent)
{
    const auto reply = ResultMessage::MakeReply(5, true, "ok", {1.0, 2.0}, {"a"});
    const std::string rj = SerializeResult(reply);
    EXPECT_NE(rj.find("\"type\":\"reply\""), std::string::npos);
    EXPECT_NE(rj.find("\"id\":5"), std::string::npos);
    EXPECT_NE(rj.find("\"success\":true"), std::string::npos);
    EXPECT_NE(rj.find("\"result\":[1.000000,2.000000]"), std::string::npos);
    EXPECT_NE(rj.find("\"strings\":[\"a\"]"), std::string::npos);

    const auto ev = ResultMessage::MakeEvent(std::string(EventName::kPlanDone), 9, true, "done");
    const std::string ej = SerializeResult(ev);
    EXPECT_NE(ej.find("\"type\":\"event\""), std::string::npos);
    EXPECT_NE(ej.find("\"ack_id\":9"), std::string::npos);
    EXPECT_NE(ej.find("\"event\":\"plan_done\""), std::string::npos);
    EXPECT_EQ(ej.find("\"ack_id\":9"), ej.rfind("\"ack_id\":9"));  // 只出现一次
}

TEST(Protocol, SerializeState)
{
    StateMessage s;
    s.timestamp = 1.5;
    s.frame_rate = 125.0;
    s.joint_pos = {0.1, 0.2};
    s.tool_index = 1;
    s.tool_pose = {0, 0, 0, 0, 0, 0};
    const std::string j = SerializeState(s);
    EXPECT_NE(j.find("\"type\":\"state\""), std::string::npos);
    EXPECT_NE(j.find("\"joint_pos\":[0.100000,0.200000]"), std::string::npos);
    EXPECT_NE(j.find("\"tool_index\":1"), std::string::npos);
}

// ── 结构化错误码：默认值 / 序列化 / 名称映射 ──
TEST(Protocol, ErrorCodeInResult)
{
    const auto ok = ResultMessage::MakeReply(1, true, "ok");
    EXPECT_EQ(ok.error_code, 0u);
    EXPECT_NE(SerializeResult(ok).find("\"error_code\":0"), std::string::npos);

    const auto bad = ResultMessage::MakeReply(2, false, "unknown command: x", {}, {},
                                              to_u32(ErrorCode::kUnknownCommand));
    EXPECT_EQ(bad.error_code, 1001u);
    EXPECT_NE(SerializeResult(bad).find("\"error_code\":1001"), std::string::npos);

    const auto ev = ResultMessage::MakeEvent("error", 3, false, "timeout", {}, {},
                                             to_u32(ErrorCode::kTimeout));
    EXPECT_EQ(ev.error_code, 1004u);
    EXPECT_NE(SerializeResult(ev).find("\"error_code\":1004"), std::string::npos);

    EXPECT_STREQ(error_code_name(ErrorCode::kModuleFailure), "MODULE_FAILURE");
}

// ────────────────────────────────────────────────────────────────
//  command_types：指令解析（含 P0 新增的 pre_scan_done）
// ────────────────────────────────────────────────────────────────
TEST(CommandParse, NoArgCommands)
{
    std::string err;
    auto v = Cmd::ParseCommand("pre_scan_done", {}, "", err);
    ASSERT_TRUE(v.has_value()) << err;
    EXPECT_EQ(std::string(Cmd::NameOf(*v)), std::string(CmdName::kPreScanDone));

    err.clear();
    v = Cmd::ParseCommand("plan", {}, "", err);
    ASSERT_TRUE(v.has_value()) << err;
    EXPECT_EQ(std::string(Cmd::NameOf(*v)), std::string(CmdName::kPlan));

    // 无参指令带参数 → 非法
    err.clear();
    EXPECT_FALSE(Cmd::ParseCommand("plan", {1.0}, "", err).has_value());
    EXPECT_NE(err.find("invalid args"), std::string::npos);
}

TEST(CommandParse, ArgCommands)
{
    std::string err;
    auto v = Cmd::ParseCommand("set_start_pose", {1.0, 2.0, 3.0}, "", err);
    ASSERT_TRUE(v.has_value()) << err;

    err.clear();
    EXPECT_FALSE(Cmd::ParseCommand("set_start_pose", {1.0, 2.0}, "", err).has_value());
    EXPECT_NE(err.find("invalid args"), std::string::npos);

    err.clear();
    v = Cmd::ParseCommand("movel", {1.0, 2.0, 3.0}, "", err);
    EXPECT_TRUE(v.has_value()) << err;
}

TEST(CommandParse, UnknownCommand)
{
    std::string err;
    EXPECT_FALSE(Cmd::ParseCommand("definitely_not_a_command", {}, "", err).has_value());
    EXPECT_NE(err.find("unknown command"), std::string::npos);
}

// ────────────────────────────────────────────────────────────────
//  utils：位姿转换
// ────────────────────────────────────────────────────────────────
TEST(Utils, MakePosePositionOnly)
{
    const auto p = MakePose({1.0, 2.0, 3.0});
    EXPECT_DOUBLE_EQ(p.position.x, 1.0);
    EXPECT_DOUBLE_EQ(p.position.y, 2.0);
    EXPECT_DOUBLE_EQ(p.position.z, 3.0);
    EXPECT_DOUBLE_EQ(p.orientation.w, 1.0);
}

TEST(Utils, MakePoseRPYRoundTrip)
{
    const double rx = 0.1, ry = -0.2, rz = 0.3;
    const auto p = MakePose({1.0, 2.0, 3.0, rx, ry, rz});
    double ox, oy, oz;
    PoseToRPY(p, ox, oy, oz);
    EXPECT_NEAR(ox, rx, 1e-9);
    EXPECT_NEAR(oy, ry, 1e-9);
    EXPECT_NEAR(oz, rz, 1e-9);
}

TEST(Utils, MakePoseQuaternion)
{
    // 位置 + 四元数（单位）应保持姿态单位
    const auto p = MakePose({1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0});
    EXPECT_DOUBLE_EQ(p.orientation.w, 1.0);
    EXPECT_DOUBLE_EQ(p.orientation.x, 0.0);
}

TEST(Utils, MatrixPoseRoundTrip)
{
    const auto p = MakePose({0.5, -1.0, 2.0, 0.2, 0.4, -0.1});
    const auto recovered = Matrix4dToPose(PoseToMatrix4d(p));
    EXPECT_NEAR(recovered.position.x, p.position.x, 1e-9);
    EXPECT_NEAR(recovered.position.z, p.position.z, 1e-9);
    double rx1, ry1, rz1, rx2, ry2, rz2;
    PoseToRPY(p, rx1, ry1, rz1);
    PoseToRPY(recovered, rx2, ry2, rz2);
    EXPECT_NEAR(rx1, rx2, 1e-9);
    EXPECT_NEAR(ry1, ry2, 1e-9);
    EXPECT_NEAR(rz1, rz2, 1e-9);
}

TEST(Utils, FlangePosToPose)
{
    const auto p = FlangePosToPose({1.0, 2.0, 3.0, 0.1, -0.2, 0.3});
    EXPECT_DOUBLE_EQ(p.position.x, 1.0);
    EXPECT_DOUBLE_EQ(p.position.y, 2.0);
    EXPECT_DOUBLE_EQ(p.position.z, 3.0);
    double rx, ry, rz;
    PoseToRPY(p, rx, ry, rz);
    EXPECT_NEAR(rx, 0.1, 1e-9);
    EXPECT_NEAR(ry, -0.2, 1e-9);
    EXPECT_NEAR(rz, 0.3, 1e-9);

    // 仅位置 → 单位姿态
    const auto p2 = FlangePosToPose({4.0, 5.0, 6.0});
    EXPECT_DOUBLE_EQ(p2.orientation.w, 1.0);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
