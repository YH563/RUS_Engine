// ════════════════════════════════════════════════════════════════════
//  rus_sim_recorder 单元测试：CRC32 / 小端写入 / 记录文件读写往返
//  storage 与 components 均为纯 std，测试不依赖 ROS 运行时。
// ════════════════════════════════════════════════════════════════════

#include <gtest/gtest.h>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <sstream>
#include <string>
#include <vector>

#include <unistd.h>

#include "components/bin_writer.hpp"
#include "components/crc32.hpp"
#include "components/rec_format.hpp"
#include "storage/rec_reader.hpp"
#include "storage/rec_writer.hpp"

using namespace RusRecorder;
using Components::BinWriter;
using Components::Crc32;
using Components::Crc32Stream;
using Components::PayloadKind;

namespace {

std::string TempPath(const std::string& name)
{
    namespace fs = std::filesystem;
    static const fs::path dir =
        fs::temp_directory_path() / ("rusrec_test_" + std::to_string(::getpid()));
    fs::create_directories(dir);
    return (dir / name).string();
}

}  // namespace

// ────────────────────────────────────────────────────────────────
//  CRC32
// ────────────────────────────────────────────────────────────────
TEST(Crc32, KnownVector)
{
    const char* s = "123456789";
    EXPECT_EQ(Crc32(s, 9), 0xCBF43926u);
    EXPECT_EQ(Crc32(nullptr, 0), 0u);
}

TEST(Crc32, StreamMatchesOneShot)
{
    const std::string s = "the quick brown fox jumps over the lazy dog";
    Crc32Stream st;
    st.Update(s.data(), 10);
    st.Update(s.data() + 10, s.size() - 10);
    EXPECT_EQ(st.Value(), Crc32(s.data(), s.size()));
}

// ────────────────────────────────────────────────────────────────
//  BinWriter：小端 + 长度前缀字符串
// ────────────────────────────────────────────────────────────────
TEST(BinWriter, LittleEndianLayout)
{
    std::ostringstream os;
    BinWriter w(os, 16);
    w.U16(0x1234);
    w.U32(0x89ABCDEF);
    w.U64(0x0102030405060708ULL);
    w.Str("hi");

    EXPECT_EQ(w.Offset(), 2u + 4u + 8u + 2u + 2u);
    ASSERT_TRUE(w.Flush());

    const std::string s = os.str();
    const std::vector<uint8_t> got(s.begin(), s.end());
    const std::vector<uint8_t> want = {
        0x34, 0x12,                                     // U16
        0xEF, 0xCD, 0xAB, 0x89,                         // U32
        0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01, // U64
        0x02, 0x00, 'h', 'i'                            // Str
    };
    EXPECT_EQ(got, want);
}

// ────────────────────────────────────────────────────────────────
//  rec_format 常量
// ────────────────────────────────────────────────────────────────
TEST(RecFormat, SizesAndKindName)
{
    EXPECT_EQ(Components::kFileHeaderSize, 64);
    EXPECT_EQ(Components::kRecHeaderSize, 40);
    EXPECT_EQ(Components::kIndexEntrySize, 32);
    EXPECT_EQ(Components::kFooterSize, 32);
    EXPECT_STREQ(Components::payload_kind_name(0), "ros_msg");
    EXPECT_STREQ(Components::payload_kind_name(999), "unknown");
    EXPECT_EQ(static_cast<uint16_t>(PayloadKind::kRosMsg), 0);
}

// ────────────────────────────────────────────────────────────────
//  RecWriter → RecReader 往返
// ────────────────────────────────────────────────────────────────
namespace {

const std::vector<uint8_t> kP0 = {10, 20, 30};
const std::vector<uint8_t> kP1 = {1, 2, 3, 4, 5, 6};

void WriteSampleFile(const std::string& path)
{
    Storage::RecWriter w;
    Storage::RecWriter::Options opt;
    opt.path = path;
    ASSERT_TRUE(w.Open(opt)) << w.error();

    Components::ChannelDesc c0;
    c0.channel_id = 0;
    c0.kind = static_cast<uint16_t>(PayloadKind::kRosMsg);
    c0.topic = "/driver/state";
    c0.type_name = "rus_sim_interfaces/msg/RobotState";
    c0.note = "state";
    ASSERT_TRUE(w.AddChannel(c0));

    Components::ChannelDesc c1;
    c1.channel_id = 1;
    c1.kind = static_cast<uint16_t>(PayloadKind::kRosMsg);
    c1.topic = "/sensor/pointcloud";
    c1.type_name = "rus_sim_interfaces/msg/SensorFrame";
    c1.note = "sensor";
    ASSERT_TRUE(w.AddChannel(c1));

    ASSERT_TRUE(w.Start());
    ASSERT_TRUE(w.WriteRecord(0, 1000, 2000, kP0.data(), kP0.size()));
    ASSERT_TRUE(w.WriteRecord(1, 1001, 2001, nullptr, 0));
    ASSERT_TRUE(w.WriteRecord(1, 1002, 2002, kP1.data(), kP1.size()));
    ASSERT_TRUE(w.Close()) << w.error();
    EXPECT_EQ(w.stats().records, 3u);
    EXPECT_EQ(w.stats().payload_bytes, kP0.size() + kP1.size());
}

}  // namespace

TEST(RecFile, WriteReadRoundTrip)
{
    const std::string path = TempPath("roundtrip.rusrec");
    WriteSampleFile(path);

    Storage::RecReader r;
    ASSERT_TRUE(r.Open(path)) << r.error();
    EXPECT_EQ(r.header().channel_count, 2u);
    EXPECT_TRUE(r.has_index());
    EXPECT_EQ(r.index().size(), 3u);

    const Components::ChannelDesc* ch0 = r.channel(0);
    ASSERT_NE(ch0, nullptr);
    EXPECT_EQ(ch0->topic, "/driver/state");
    EXPECT_EQ(ch0->type_name, "rus_sim_interfaces/msg/RobotState");

    // 顺序扫描 + CRC
    int count = 0;
    uint64_t bytes = 0;
    const bool ok = r.Scan([&](const Components::RecHeader& h, const uint8_t*) {
        ++count;
        bytes += h.payload_size;
        return true;
    }, true);
    ASSERT_TRUE(ok) << r.error();
    EXPECT_EQ(count, 3);
    EXPECT_EQ(bytes, kP0.size() + kP1.size());
    EXPECT_EQ(r.scanned_records(), 3u);
    EXPECT_EQ(r.corrupt_records(), 0u);

    // 按索引随机访问第 3 条（channel 1 的第 2 条，seq=1）
    Storage::RecReader::RecordInfo info;
    std::vector<uint8_t> payload;
    ASSERT_TRUE(r.ReadAt(2, info, payload, true)) << r.error();
    EXPECT_EQ(info.header.channel_id, 1u);
    EXPECT_EQ(info.header.seq, 1u);
    EXPECT_EQ(payload, kP1);

    // 回放路径：BuildTimeline + ReadRecordAt
    std::vector<Storage::RecReader::RecordMeta> tl;
    ASSERT_TRUE(r.BuildTimeline(tl)) << r.error();
    ASSERT_EQ(tl.size(), 3u);
    Storage::RecReader::RecordInfo info2;
    std::vector<uint8_t> payload2;
    ASSERT_TRUE(r.ReadRecordAt(tl[0].offset, info2, payload2, true)) << r.error();
    EXPECT_EQ(payload2, kP0);
}

TEST(RecFile, DetectsCorruptPayload)
{
    const std::string path = TempPath("corrupt.rusrec");
    WriteSampleFile(path);

    // 打开定位第 3 条记录的 payload 起始，翻一个字节
    int64_t payload_off = -1;
    {
        Storage::RecReader r;
        ASSERT_TRUE(r.Open(path)) << r.error();
        ASSERT_EQ(r.index().size(), 3u);
        payload_off = r.index()[2].offset + Components::kRecHeaderSize;
    }

    std::vector<uint8_t> bytes;
    {
        std::ifstream in(path, std::ios::binary);
        bytes.assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
    }
    ASSERT_GT(static_cast<int64_t>(bytes.size()), payload_off);
    bytes[payload_off] ^= 0xFF;
    {
        std::ofstream out(path, std::ios::binary | std::ios::trunc);
        out.write(reinterpret_cast<const char*>(bytes.data()),
                  static_cast<std::streamsize>(bytes.size()));
    }

    Storage::RecReader r2;
    ASSERT_TRUE(r2.Open(path)) << r2.error();
    int count = 0;
    ASSERT_TRUE(r2.Scan([&](const Components::RecHeader&, const uint8_t*) { ++count; return true; }, true));
    EXPECT_EQ(count, 3);
    EXPECT_EQ(r2.corrupt_records(), 1u);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
