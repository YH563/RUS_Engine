#pragma once

// ════════════════════════════════════════════════════════════════════
//  记录文件格式定义（components：纯格式常量 + 结构体，无逻辑无依赖）
//  ────────────────────────────────────────────────────────────────────
//  单文件 `.rusrec` 布局（全部小端、紧凑排列，逐字段显式编解码，
//  不依赖主机字节序与编译器 padding）：
//
//    ┌──────────────┬─────────────────────────┬──────────────┬───────────┐
//    │ FileHeader   │ ChannelDesc × channel_count │ Record × N │ Index + Footer │
//    │  64 B 固定   │  变长（u16 长度前缀字符串）  │  40 B + payload │  32 B × N + 32 B │
//    └──────────────┴─────────────────────────┴──────────────┴───────────┘
//
//  设计要点（详见 docs/Protocol/RecFormat.md）：
//    1. 每条记录自带魔数 `REC1`：崩溃文件（无尾索引）也能顺序扫描重建；
//    2. payload 自带 CRC32：写入截断 / 磁盘坏块在读时即被发现；
//    3. 通道表带话题名 + 消息类型名：payload 是 ROS 消息的 CDR 字节，
//       离线工具按 type_name 反序列化即可还原（无需 rosbag2）；
//    4. 结构尺寸写进文件头（header_size / rec_header_size / index_entry_size），
//       将来加字段时老工具能按尺寸跳过未知区域。
// ════════════════════════════════════════════════════════════════════

#include <cstdint>
#include <string>

namespace RusRecorder::Components {

    // ── 魔数 / 版本（按字节定义，避免端序歧义）──
    inline constexpr char kFileMagic[8]   = {'R', 'U', 'S', 'R', 'E', 'C', 'v', '1'};
    inline constexpr char kRecMagic[4]    = {'R', 'E', 'C', '1'};
    inline constexpr char kFooterMagic[8] = {'R', 'U', 'S', 'I', 'D', 'X', '0', '1'};

    inline constexpr uint16_t kFormatVersion = 1;

    // ── 定长结构尺寸 ──
    inline constexpr uint16_t kFileHeaderSize = 64;
    inline constexpr uint16_t kRecHeaderSize  = 40;
    inline constexpr uint16_t kIndexEntrySize = 32;
    inline constexpr uint16_t kFooterSize     = 32;

    /// 单条 payload 上限（读侧防御：头部损坏时不至于按天文数字分配内存）
    inline constexpr uint32_t kMaxPayloadSize = 256u * 1024u * 1024u;  // 256 MiB

    // ── 字段分解式自校验：编解码顺序必须与上面的尺寸常量一致 ──
    //   FileHeader : magic(8) ver(2) hdr(2) rechdr(2) idx(2) flags(4)
    //                created_ns(8) ch_count(2) reserved(2) reserved2(32)
    //   RecHeader  : magic(4) channel(2) kind(2) seq(4) stamp(8) recv(8)
    //                payload_size(4) crc(4) reserved(4)
    //   IndexEntry : offset(8) stamp(8) channel(2) kind(2) size(4) crc(4) reserved(4)
    //   FileFooter : index_offset(8) index_count(8) file_size(8) magic(8)
    static_assert(kFileHeaderSize == 8 + 2 + 2 + 2 + 2 + 4 + 8 + 2 + 2 + 32, "FileHeader 字段与尺寸不符");
    static_assert(kRecHeaderSize == 4 + 2 + 2 + 4 + 8 + 8 + 4 + 4 + 4, "RecHeader 字段与尺寸不符");
    static_assert(kIndexEntrySize == 8 + 8 + 2 + 2 + 4 + 4 + 4, "IndexEntry 字段与尺寸不符");
    static_assert(kFooterSize == 8 + 8 + 8 + 8, "FileFooter 字段与尺寸不符");

    /// payload 编码方式（ChannelDesc::kind / RecHeader::kind）
    enum class PayloadKind : uint16_t {
        kRosMsg = 0,  // ROS 消息的 CDR 序列化字节（rclcpp::Serialization），类型见 type_name
    };

    /// kind → 人类可读名（inspect 打印用）
    inline const char* payload_kind_name(uint16_t kind)
    {
        switch (static_cast<PayloadKind>(kind)) {
            case PayloadKind::kRosMsg: return "ros_msg";
        }
        return "unknown";
    }

    /// 文件头（文件起始，固定 64 字节）
    struct FileHeader {
        uint16_t version = kFormatVersion;
        uint16_t header_size = kFileHeaderSize;
        uint16_t rec_header_size = kRecHeaderSize;
        uint16_t index_entry_size = kIndexEntrySize;
        uint32_t flags = 0;              // 预留（写 0，读侧忽略）
        int64_t created_unix_ns = 0;     // 录制开始时刻（本机系统时钟）
        uint16_t channel_count = 0;      // 紧随其后的 ChannelDesc 条数
    };

    /// 通道描述（FileHeader 之后，每条变长；同一 channel_id 只出现一次）
    struct ChannelDesc {
        uint16_t channel_id = 0;                          // 0 = 机械臂状态，1 = 感知帧（见 docs）
        uint16_t kind = static_cast<uint16_t>(PayloadKind::kRosMsg);
        std::string topic;      // 记录来源话题，如 /driver/state
        std::string type_name;  // 消息类型，如 rus_sim_interfaces/msg/RobotState
        std::string note;       // 人类可读说明（inspect 打印用）
    };

    /// 记录头（每条记录固定 40 字节）
    struct RecHeader {
        uint16_t channel_id = 0;
        uint16_t kind = 0;
        uint32_t seq = 0;            // 通道内自增（从 0）：跳变 = 丢记录或丢文件段
        int64_t stamp_ns = 0;        // 消息时间戳（ROS 时间；0 = 消息未带时间戳）
        int64_t recv_ns = 0;         // 入队时刻（本机系统时钟）
        uint32_t payload_size = 0;
        uint32_t payload_crc32 = 0;  // payload 字节 CRC32（payload_size=0 时为 0）
    };

    /// 尾索引项（索引区，每条记录一项，固定 32 字节）
    struct IndexEntry {
        int64_t offset = 0;      // 记录头在文件中的绝对偏移
        int64_t stamp_ns = 0;
        uint16_t channel_id = 0;
        uint16_t kind = 0;
        uint32_t payload_size = 0;
        uint32_t payload_crc32 = 0;
    };

    /// 文件尾（索引区之后，固定 32 字节；缺失 = 文件未正常关闭）
    struct FileFooter {
        int64_t index_offset = 0;  // 索引区起始偏移
        int64_t index_count = 0;   // 索引项条数（= 记录总数）
        int64_t file_size = 0;     // 文件总字节数（含本 Footer）
    };

}  // namespace RusRecorder::Components
