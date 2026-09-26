#pragma once

// ════════════════════════════════════════════════════════════════════
//  记录读取器（storage：文件体检 / 离线回放共用）
//  ────────────────────────────────────────────────────────────────────
//  三条读取路径，互为校验：
//    1. LoadIndex()：读文件尾 FileFooter + 索引区 → 随机访问（ReadAt）+ 免扫描统计；
//       仅对"正常关闭"的文件可用。
//    2. Scan()：从记录流起点顺序扫描逐条解析（校验魔数 / 长度 / 可选 CRC），
//       用于崩溃文件（无尾索引）与"索引 ↔ 实际内容"一致性校验。
//    3. BuildTimeline() + ReadRecordAt()：**回放专用** —— 顺序扫描只收集记录元数据
//       （不读 payload 内容，故大文件也很快），再按元数据里的偏移逐条读回；
//       有 / 无尾索引的文件都能用（崩溃录音同样可回放 + 可 seek）。
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

        /// 记录元数据（时间轴条目：不含 payload 内容，回放按 offset 惰性读回）
        struct RecordMeta {
            int64_t offset = 0;        // 记录头绝对偏移（ReadRecordAt 入参）
            uint16_t channel_id = 0;
            uint16_t kind = 0;
            uint32_t seq = 0;
            int64_t stamp_ns = 0;      // 消息时间戳（0 = 消息未带时间戳）
            int64_t recv_ns = 0;       // 录制入队时刻（本机系统时钟）
            uint32_t payload_size = 0;
            uint32_t payload_crc32 = 0;
        };

        /// 记录回调：返回 false 可提前终止扫描
        using RecordCallback = std::function<bool(const Components::RecHeader&, const uint8_t* payload)>;

        /// 打开并解析头部 / 通道表，尝试加载尾索引（失败不报错：has_index()=false）
        bool Open(const std::string& path, std::string* err = nullptr);

        /// 关闭底层文件（释放句柄；Open() 内部也会先关一次，可安全地反复换文件）
        void Close();

        /// 顺序扫描全部记录（check_crc=true 时逐条校验 payload CRC）
        bool Scan(const RecordCallback& cb, bool check_crc, std::string* err = nullptr);

        /**
         * @brief 顺序扫描收集记录元数据（时间轴；**不读 payload 内容**，大文件也快）
         *
         * 与 Scan() 共用同一套"记录流边界"规则：有尾索引时扫到索引区前收住，
         * 无尾索引（崩溃 / 截断）时扫到末尾残缺处收住并写 error()。
         * 同时更新 scanned_records() / scan_end_offset() / channel_stats()（seq 跳变、
         * 各通道条数与字节数）——但**不校验 payload CRC**（payload 没被读），
         * 因此 corrupt_records() 保持 0；CRC 校验放在回放发布时逐条做（ReadRecordAt）。
         *
         * @param out 输出时间轴（按记录写入顺序，即时间顺序；调用前内容被清空）
         */
        bool BuildTimeline(std::vector<RecordMeta>& out, std::string* err = nullptr);

        /// 按尾索引读取第 i 条记录（随机访问）
        bool ReadAt(size_t i, RecordInfo& info, std::vector<uint8_t>& payload, bool check_crc);

        /**
         * @brief 按记录偏移读取一条记录（回放随机访问；不依赖尾索引）
         * @param offset 记录头绝对偏移（来自 BuildTimeline 的 RecordMeta::offset）
         */
        bool ReadRecordAt(int64_t offset, RecordInfo& info, std::vector<uint8_t>& payload,
                          bool check_crc);

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

        /// 按偏移读一条记录（ReadAt / ReadRecordAt 共用；expected != nullptr = 校验索引 ↔ 记录一致）
        bool read_record_at(int64_t offset, RecordInfo& info, std::vector<uint8_t>& payload,
                            bool check_crc, const Components::IndexEntry* expected);

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
