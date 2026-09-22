#pragma once

// ════════════════════════════════════════════════════════════════════
//  记录读取器（storage：文件体检 / 离线回放共用）
//  ────────────────────────────────────────────────────────────────────
//  两条读取路径，互为校验：
//    1. LoadIndex()：读文件尾 FileFooter + 索引区 → 随机访问（ReadAt）+ 免扫描统计；
//       仅对"正常关闭"的文件可用。
//    2. Scan()：从记录流起点顺序扫描逐条解析（校验魔数 / 长度 / 可选 CRC），
//       用于崩溃文件（无尾索引）与"索引 ↔ 实际内容"一致性校验。
//
//  纯 std 实现（无 ROS 类型）：工具与单测可在无 ROS 环境下运行。
//  线程模型：非线程安全（单线程使用）。
// ════════════════════════════════════════════════════════════════════

#include <cstdint>
#include <fstream>
#include <functional>
#include <string>
#include <vector>

#include "components/rec_format.hpp"

namespace RusRecorder::Storage {

    /// `.rusrec` 只读访问器
    class RecReader {
    public:
        /// 每通道扫描统计（Scan() 后有效）
        struct ChannelStats {
            uint64_t records = 0;
            uint64_t payload_bytes = 0;
            uint64_t corrupt = 0;          // CRC 不符的记录数
            uint64_t seq_gaps = 0;         // seq 跳变次数（丢记录 / 丢文件段）
            uint32_t max_payload_size = 0;
            int64_t first_stamp_ns = 0;
            int64_t last_stamp_ns = 0;
        };

        /// 单条记录定位结果（按索引随机访问）
        struct RecordInfo {
            Components::RecHeader header;
            int64_t offset = 0;  // 记录头绝对偏移
        };

        /// 记录回调：返回 false 可提前终止扫描
        using RecordCallback = std::function<bool(const Components::RecHeader&, const uint8_t* payload)>;

        /// 打开并解析头部 / 通道表，尝试加载尾索引（失败不报错：has_index()=false）
        bool Open(const std::string& path, std::string* err = nullptr);

        /// 顺序扫描全部记录（check_crc=true 时逐条校验 payload CRC）
        bool Scan(const RecordCallback& cb, bool check_crc, std::string* err = nullptr);

        /// 按尾索引读取第 i 条记录（随机访问，供将来离线回放 seek）
        bool ReadAt(size_t i, RecordInfo& info, std::vector<uint8_t>& payload, bool check_crc);

        const Components::FileHeader& header() const { return header_; }
        const std::vector<Components::ChannelDesc>& channels() const { return channels_; }
        const std::vector<Components::IndexEntry>& index() const { return index_; }
        /// 是否读到完整尾索引（= 文件正常关闭）
        bool has_index() const { return has_index_; }
        /// 尾索引自洽性（offset 单调、条数 / 文件尺寸吻合）
        bool index_sane() const { return index_sane_; }
        /// Scan() 后的各通道统计
        const std::vector<ChannelStats>& channel_stats() const { return stats_; }
        /// Scan() 覆盖到的字节数（未覆盖的尾部 = 未写完的记录 / 截断残留）
        uint64_t scan_end_offset() const { return scan_end_offset_; }
        int64_t file_size() const { return file_size_; }
        /// 扫描到的记录总数
        uint64_t scanned_records() const { return scanned_records_; }
        /// 扫描中 CRC 不符的记录数（check_crc=false 时不计）
        uint64_t corrupt_records() const { return corrupt_records_; }
        /// 按 channel_id 查通道描述（未知返回 nullptr）
        const Components::ChannelDesc* channel(uint16_t id) const;
        const std::string& error() const { return error_; }

    private:
        bool load_file_header(std::string* err);
        bool load_channels(std::string* err);
        bool try_load_index();

        std::ifstream is_;
        uint64_t stream_offset_ = 0;  // 记录流起点（FileHeader + 通道表之后）
        int64_t index_offset_hint_ = -1;  // 尾索引起始偏移（读到 Footer 后填写；<0 = 未知）
        std::string path_;
        Components::FileHeader header_;
        std::vector<Components::ChannelDesc> channels_;
        std::vector<Components::IndexEntry> index_;
        std::vector<ChannelStats> stats_;
        std::string error_;
        int64_t file_size_ = 0;
        uint64_t scan_end_offset_ = 0;
        uint64_t scanned_records_ = 0;
        uint64_t corrupt_records_ = 0;
        bool has_index_ = false;
        bool index_sane_ = false;
    };

}  // namespace RusRecorder::Storage
