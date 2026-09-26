#include "storage/rec_reader.hpp"

#include <algorithm>
#include <cstring>
#include <map>
#include <string>

#include "components/crc32.hpp"

namespace RusRecorder::Storage {

    namespace {
        // ── 小端读取基元（与 components/bin_writer.hpp 的写侧一一对应）──
        inline uint16_t RdU16(const uint8_t* p)
        {
            return static_cast<uint16_t>(p[0] | (static_cast<uint16_t>(p[1]) << 8));
        }

        inline uint32_t RdU32(const uint8_t* p)
        {
            return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
                   (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
        }

        inline uint64_t RdU64(const uint8_t* p)
        {
            uint64_t v = 0;
            for (int i = 7; i >= 0; --i) v = (v << 8) | p[i];
            return v;
        }

        inline int64_t RdI64(const uint8_t* p) { return static_cast<int64_t>(RdU64(p)); }

        /// u16 长度前缀字符串（pos 前进；越界返回 false）
        bool RdStr(const uint8_t* buf, size_t len, size_t& pos, std::string& out)
        {
            if (pos + 2 > len) return false;
            const uint16_t n = RdU16(buf + pos);
            pos += 2;
            if (pos + n > len) return false;
            out.assign(reinterpret_cast<const char*>(buf + pos), n);
            pos += n;
            return true;
        }

        /// 通道表读取上限（防伪造 channel_count 造成大分配）
        constexpr size_t kChannelTableMaxBytes = 64u * 1024u;
        /// 尾索引读取上限（超过则视为不可信，退化为顺序扫描）
        constexpr int64_t kIndexMaxBytes = 256LL * 1024 * 1024;
    }  // namespace

    void RecReader::Close()
    {
        if (is_.is_open()) is_.close();
        is_.clear();
    }

    bool RecReader::Open(const std::string& path, std::string* err)
    {
        path_ = path;
        error_.clear();
        // 反复 Open（回放换文件）必须先关掉上一个句柄：ifstream::open 对已打开的流会置 failbit
        Close();
        channels_.clear();
        index_.clear();
        stats_.clear();
        has_index_ = false;
        index_sane_ = false;
        scanned_records_ = 0;
        corrupt_records_ = 0;
        scan_end_offset_ = 0;
        stream_offset_ = 0;
        index_offset_hint_ = -1;
        file_size_ = 0;

        is_.open(path_, std::ios::binary);
        if (!is_.is_open()) {
            error_ = "无法打开文件：" + path_;
            if (err) *err = error_;
            return false;
        }
        is_.seekg(0, std::ios::end);
        file_size_ = static_cast<int64_t>(is_.tellg());
        is_.clear();

        if (!load_file_header(err)) return false;
        if (!load_channels(err)) return false;
        try_load_index();  // 失败不算错误：has_index() = false，改用 Scan()
        return true;
    }

    bool RecReader::load_file_header(std::string* err)
    {
        uint8_t buf[Components::kFileHeaderSize] = {0};
        is_.clear();
        is_.seekg(0);
        is_.read(reinterpret_cast<char*>(buf), sizeof(buf));
        if (is_.gcount() != static_cast<std::streamsize>(sizeof(buf))) {
            error_ = "文件过短（不足 " + std::to_string(sizeof(buf)) + " 字节）";
            if (err) *err = error_;
            return false;
        }
        if (std::memcmp(buf, Components::kFileMagic, sizeof(Components::kFileMagic)) != 0) {
            error_ = "魔数不符（不是 .rusrec 文件）";
            if (err) *err = error_;
            return false;
        }

        size_t pos = sizeof(Components::kFileMagic);
        header_.version = RdU16(buf + pos);           pos += 2;
        header_.header_size = RdU16(buf + pos);       pos += 2;
        header_.rec_header_size = RdU16(buf + pos);   pos += 2;
        header_.index_entry_size = RdU16(buf + pos);  pos += 2;
        header_.flags = RdU32(buf + pos);             pos += 4;
        header_.created_unix_ns = RdI64(buf + pos);   pos += 8;
        header_.channel_count = RdU16(buf + pos);     pos += 2;
        // reserved(2) + reserved2(32)：预留位，读侧忽略

        if (header_.version != Components::kFormatVersion) {
            error_ = "不支持的格式版本：" + std::to_string(header_.version) +
                     "（本工具支持 v" + std::to_string(Components::kFormatVersion) + "）";
            if (err) *err = error_;
            return false;
        }
        if (header_.header_size < Components::kFileHeaderSize ||
            header_.rec_header_size < Components::kRecHeaderSize ||
            header_.index_entry_size < Components::kIndexEntrySize) {
            error_ = "头部声明的结构尺寸小于支持的最小值（文件损坏？）";
            if (err) *err = error_;
            return false;
        }
        return true;
    }

    bool RecReader::load_channels(std::string* err)
    {
        if (header_.channel_count == 0) {
            error_ = "通道表为空（文件损坏？）";
            if (err) *err = error_;
            return false;
        }
        const int64_t avail = file_size_ - static_cast<int64_t>(header_.header_size);
        if (avail <= 0) {
            error_ = "通道表越界（文件损坏？）";
            if (err) *err = error_;
            return false;
        }

        const size_t chunk = std::min<size_t>(kChannelTableMaxBytes, static_cast<size_t>(avail));
        std::vector<uint8_t> buf(chunk);
        is_.clear();
        is_.seekg(static_cast<std::streamoff>(header_.header_size));
        is_.read(reinterpret_cast<char*>(buf.data()), static_cast<std::streamsize>(chunk));
        buf.resize(static_cast<size_t>(std::max<std::streamsize>(0, is_.gcount())));

        size_t pos = 0;
        for (uint16_t i = 0; i < header_.channel_count; ++i) {
            if (pos + 4 > buf.size()) {
                error_ = "通道表截断（第 " + std::to_string(i + 1) + " 条起越界）";
                if (err) *err = error_;
                return false;
            }
            Components::ChannelDesc desc;
            desc.channel_id = RdU16(buf.data() + pos); pos += 2;
            desc.kind = RdU16(buf.data() + pos);       pos += 2;
            if (!RdStr(buf.data(), buf.size(), pos, desc.topic) ||
                !RdStr(buf.data(), buf.size(), pos, desc.type_name) ||
                !RdStr(buf.data(), buf.size(), pos, desc.note)) {
                error_ = "通道表字符串截断（第 " + std::to_string(i + 1) + " 条起越界）";
                if (err) *err = error_;
                return false;
            }
            channels_.push_back(desc);
        }
        stream_offset_ = header_.header_size + pos;  // 记录流起点
        stats_.resize(channels_.size());
        return true;
    }

    bool RecReader::try_load_index()
    {
        has_index_ = false;
        index_sane_ = false;
        if (file_size_ < static_cast<int64_t>(Components::kFooterSize)) return false;

        uint8_t foot[Components::kFooterSize] = {0};
        is_.clear();
        is_.seekg(static_cast<std::streamoff>(file_size_ - Components::kFooterSize));
        is_.read(reinterpret_cast<char*>(foot), sizeof(foot));
        if (is_.gcount() != static_cast<std::streamsize>(sizeof(foot))) return false;
        if (std::memcmp(foot + 24, Components::kFooterMagic,
                        sizeof(Components::kFooterMagic)) != 0) {
            return false;  // 无 Footer → 文件未正常关闭（进程被强杀）
        }

        const int64_t index_offset = RdI64(foot + 0);
        const int64_t index_count = RdI64(foot + 8);
        const int64_t file_size = RdI64(foot + 16);
        if (index_count < 0 || index_offset < static_cast<int64_t>(stream_offset_)) return false;
        if (file_size != file_size_) return false;  // Footer 与真实尺寸不符
        // 记录流到此为止（顺序扫描据此在索引区前收住，避免把索引当成记录读）
        index_offset_hint_ = index_offset;
        // 防溢出 / 防伪造：索引条数不可能超过"每条占满 32 字节"的上界
        if (index_count > file_size_ / Components::kIndexEntrySize + 1) return false;
        const int64_t index_bytes = index_count * static_cast<int64_t>(header_.index_entry_size);
        if (index_bytes > kIndexMaxBytes) return false;
        if (index_offset + index_bytes + static_cast<int64_t>(Components::kFooterSize) > file_size_) {
            return false;
        }

        std::vector<uint8_t> raw(static_cast<size_t>(index_bytes));
        is_.clear();
        is_.seekg(static_cast<std::streamoff>(index_offset));
        is_.read(reinterpret_cast<char*>(raw.data()), static_cast<std::streamsize>(raw.size()));
        if (is_.gcount() != static_cast<std::streamsize>(raw.size())) return false;

        index_.clear();
        index_.reserve(static_cast<size_t>(index_count));
        const int64_t stride = static_cast<int64_t>(header_.index_entry_size);
        for (int64_t i = 0; i < index_count; ++i) {
            const uint8_t* p = raw.data() + static_cast<size_t>(i * stride);
            Components::IndexEntry e;
            e.offset = RdI64(p + 0);
            e.stamp_ns = RdI64(p + 8);
            e.channel_id = RdU16(p + 16);
            e.kind = RdU16(p + 18);
            e.payload_size = RdU32(p + 20);
            e.payload_crc32 = RdU32(p + 24);
            index_.push_back(e);
        }
        has_index_ = true;

        // ── 索引自洽性：offset 严格递增、不早于记录流起点、末条恰好接到索引区 ──
        index_sane_ = true;
        for (size_t i = 1; i < index_.size(); ++i) {
            if (index_[i].offset <= index_[i - 1].offset) {
                index_sane_ = false;
                break;
            }
        }
        if (index_sane_ && !index_.empty()) {
            const auto& last = index_.back();
            if (last.offset < static_cast<int64_t>(stream_offset_)) {
                index_sane_ = false;
            } else if (last.offset + static_cast<int64_t>(header_.rec_header_size) +
                           static_cast<int64_t>(last.payload_size) != index_offset) {
                index_sane_ = false;
            }
        }
        return has_index_;
    }

    bool RecReader::Scan(const RecordCallback& cb, bool check_crc, std::string* err)
    {
        error_.clear();
        if (!is_.is_open()) {
            error_ = "未打开文件";
            if (err) *err = error_;
            return false;
        }

        std::map<uint16_t, size_t> stats_index;
        for (size_t i = 0; i < channels_.size(); ++i) stats_index[channels_[i].channel_id] = i;
        stats_.assign(channels_.size(), ChannelStats{});
        std::map<uint16_t, uint32_t> last_seq;

        const int64_t stride = header_.rec_header_size;
        const int64_t extra = stride - static_cast<int64_t>(Components::kRecHeaderSize);
        uint64_t offset = stream_offset_;
        std::vector<uint8_t> payload;
        scanned_records_ = 0;
        corrupt_records_ = 0;
        scan_end_offset_ = stream_offset_;

        is_.clear();
        is_.seekg(static_cast<std::streamoff>(stream_offset_));
        while (true) {
            // 有尾索引的文件：记录流到索引区前结束（index_offset 之后的字节不是记录）
            if (index_offset_hint_ >= 0 && offset >= static_cast<uint64_t>(index_offset_hint_)) break;

            uint8_t hdr[Components::kRecHeaderSize] = {0};
            is_.read(reinterpret_cast<char*>(hdr), sizeof(hdr));
            const auto got = is_.gcount();
            if (got == 0) break;                                            // 干净结束
            if (got != static_cast<std::streamsize>(sizeof(hdr))) break;    // 尾部残留（不足一个记录头）

            if (std::memcmp(hdr, Components::kRecMagic, sizeof(Components::kRecMagic)) != 0) {
                error_ = "记录魔数不符 @offset " + std::to_string(offset) +
                         "（文件截断 / 损坏，仅统计到此处）";
                break;
            }

            Components::RecHeader rh;
            rh.channel_id = RdU16(hdr + 4);    // 0..3 = magic "REC1"
            rh.kind = RdU16(hdr + 6);
            rh.seq = RdU32(hdr + 8);
            rh.stamp_ns = RdI64(hdr + 12);
            rh.recv_ns = RdI64(hdr + 20);
            rh.payload_size = RdU32(hdr + 28);
            rh.payload_crc32 = RdU32(hdr + 32);

            if (rh.payload_size > Components::kMaxPayloadSize) {
                error_ = "payload 长度异常（" + std::to_string(rh.payload_size) +
                         " B）@offset " + std::to_string(offset);
                break;
            }
            if (extra > 0) is_.seekg(extra, std::ios::cur);  // 未来格式：跳过未知字段
            payload.resize(rh.payload_size);
            if (rh.payload_size > 0) {
                is_.read(reinterpret_cast<char*>(payload.data()),
                         static_cast<std::streamsize>(rh.payload_size));
                if (is_.gcount() != static_cast<std::streamsize>(rh.payload_size)) {
                    error_ = "payload 截断 @offset " + std::to_string(offset);
                    break;
                }
            }

            const bool corrupt =
                check_crc && (Components::Crc32(payload.data(), payload.size()) != rh.payload_crc32);
            if (corrupt) corrupt_records_++;

            const auto it = stats_index.find(rh.channel_id);
            if (it != stats_index.end()) {
                ChannelStats& st = stats_[it->second];
                const auto seq_it = last_seq.find(rh.channel_id);
                if (seq_it != last_seq.end() && rh.seq != seq_it->second + 1) st.seq_gaps++;
                last_seq[rh.channel_id] = rh.seq;
                if (st.records == 0) st.first_stamp_ns = rh.stamp_ns;
                st.last_stamp_ns = rh.stamp_ns;
                st.records++;
                st.payload_bytes += rh.payload_size;
                st.max_payload_size = std::max(st.max_payload_size, rh.payload_size);
                if (corrupt) st.corrupt++;
            }

            offset += stride + rh.payload_size;
            scanned_records_++;
            scan_end_offset_ = offset;

            if (cb && !cb(rh, payload.data())) break;  // 回调提前终止（不算错误）
        }

        if (err) *err = error_;
        return true;
    }

    bool RecReader::BuildTimeline(std::vector<RecordMeta>& out, std::string* err)
    {
        error_.clear();
        out.clear();
        if (!is_.is_open()) {
            error_ = "未打开文件";
            if (err) *err = error_;
            return false;
        }

        // 与 Scan() 同口径的统计（唯一区别：payload 只跳过、不读内容）
        std::map<uint16_t, size_t> stats_index;
        for (size_t i = 0; i < channels_.size(); ++i) stats_index[channels_[i].channel_id] = i;
        stats_.assign(channels_.size(), ChannelStats{});
        std::map<uint16_t, uint32_t> last_seq;

        const int64_t stride = header_.rec_header_size;
        const int64_t extra = stride - static_cast<int64_t>(Components::kRecHeaderSize);
        // 记录流上界：有尾索引 → 索引区起点；否则 → 文件尾（崩溃 / 截断文件）
        const uint64_t limit = index_offset_hint_ >= 0
            ? static_cast<uint64_t>(index_offset_hint_)
            : static_cast<uint64_t>(file_size_);
        uint64_t offset = stream_offset_;
        scanned_records_ = 0;
        corrupt_records_ = 0;   // payload 未读 → 不做 CRC 校验（见头文件说明）
        scan_end_offset_ = stream_offset_;

        is_.clear();
        is_.seekg(static_cast<std::streamoff>(stream_offset_));
        while (offset < limit) {
            uint8_t hdr[Components::kRecHeaderSize] = {0};
            is_.read(reinterpret_cast<char*>(hdr), sizeof(hdr));
            const auto got = is_.gcount();
            if (got == 0) break;                                         // 干净结束
            if (got != static_cast<std::streamsize>(sizeof(hdr))) {       // 尾部残留（不足一个记录头）
                error_ = "记录头截断 @offset " + std::to_string(offset) +
                         "（尾部落盘不全，仅扫描到此）";
                break;
            }
            if (std::memcmp(hdr, Components::kRecMagic, sizeof(Components::kRecMagic)) != 0) {
                error_ = "记录魔数不符 @offset " + std::to_string(offset) +
                         "（文件截断 / 损坏，仅扫描到此）";
                break;
            }

            Components::RecHeader rh;
            rh.channel_id = RdU16(hdr + 4);    // 0..3 = magic "REC1"
            rh.kind = RdU16(hdr + 6);
            rh.seq = RdU32(hdr + 8);
            rh.stamp_ns = RdI64(hdr + 12);
            rh.recv_ns = RdI64(hdr + 20);
            rh.payload_size = RdU32(hdr + 28);
            rh.payload_crc32 = RdU32(hdr + 32);

            if (rh.payload_size > Components::kMaxPayloadSize) {
                error_ = "payload 长度异常（" + std::to_string(rh.payload_size) +
                         " B）@offset " + std::to_string(offset);
                break;
            }
            // payload 不真读，故越界必须显式用长度核对（末尾残缺记录到此收住）
            if (offset + static_cast<uint64_t>(stride) + rh.payload_size > limit) {
                error_ = "payload 截断 @offset " + std::to_string(offset);
                break;
            }

            RecordMeta meta;
            meta.offset = static_cast<int64_t>(offset);
            meta.channel_id = rh.channel_id;
            meta.kind = rh.kind;
            meta.seq = rh.seq;
            meta.stamp_ns = rh.stamp_ns;
            meta.recv_ns = rh.recv_ns;
            meta.payload_size = rh.payload_size;
            meta.payload_crc32 = rh.payload_crc32;
            out.push_back(meta);

            const auto it = stats_index.find(rh.channel_id);
            if (it != stats_index.end()) {
                ChannelStats& st = stats_[it->second];
                const auto seq_it = last_seq.find(rh.channel_id);
                if (seq_it != last_seq.end() && rh.seq != seq_it->second + 1) st.seq_gaps++;
                last_seq[rh.channel_id] = rh.seq;
                if (st.records == 0) st.first_stamp_ns = rh.stamp_ns;
                st.last_stamp_ns = rh.stamp_ns;
                st.records++;
                st.payload_bytes += rh.payload_size;
                st.max_payload_size = std::max(st.max_payload_size, rh.payload_size);
            }

            // 跳过 payload：只移动读指针，不把字节读进内存
            // （回放只需要"什么时候发哪条"，内容在发布前一刻按 offset 读回）
            if (extra > 0) is_.seekg(extra, std::ios::cur);
            if (rh.payload_size > 0) {
                is_.seekg(static_cast<std::streamoff>(rh.payload_size), std::ios::cur);
            }

            offset += static_cast<uint64_t>(stride) + rh.payload_size;
            scanned_records_++;
            scan_end_offset_ = offset;
        }

        if (err) *err = error_;
        return true;
    }

    bool RecReader::ReadAt(size_t i, RecordInfo& info, std::vector<uint8_t>& payload, bool check_crc)
    {
        error_.clear();
        if (!has_index_) {
            error_ = "文件无尾索引（先 Scan() 或用 rus_sim_recorder_inspect --scan）";
            return false;
        }
        if (i >= index_.size()) {
            error_ = "索引越界：" + std::to_string(i) + " / " + std::to_string(index_.size());
            return false;
        }
        const Components::IndexEntry& e = index_[i];
        return read_record_at(e.offset, info, payload, check_crc, &e);
    }

    bool RecReader::ReadRecordAt(int64_t offset, RecordInfo& info, std::vector<uint8_t>& payload,
                                 bool check_crc)
    {
        error_.clear();
        const int64_t limit = index_offset_hint_ >= 0 ? index_offset_hint_ : file_size_;
        if (offset < static_cast<int64_t>(stream_offset_) || offset >= limit) {
            error_ = "记录偏移越界：" + std::to_string(offset) +
                     "（记录流区间 [" + std::to_string(stream_offset_) + ", " +
                     std::to_string(limit) + ")）";
            return false;
        }
        return read_record_at(offset, info, payload, check_crc, nullptr);
    }

    bool RecReader::read_record_at(int64_t offset, RecordInfo& info,
                                   std::vector<uint8_t>& payload, bool check_crc,
                                   const Components::IndexEntry* expected)
    {
        uint8_t hdr[Components::kRecHeaderSize] = {0};
        is_.clear();
        is_.seekg(static_cast<std::streamoff>(offset));
        is_.read(reinterpret_cast<char*>(hdr), sizeof(hdr));
        if (is_.gcount() != static_cast<std::streamsize>(sizeof(hdr))) {
            error_ = "读取记录头失败 @offset " + std::to_string(offset);
            return false;
        }
        if (std::memcmp(hdr, Components::kRecMagic, sizeof(Components::kRecMagic)) != 0) {
            error_ = "记录魔数不符 @offset " + std::to_string(offset) + "（该偏移处的数据已损坏）";
            return false;
        }

        Components::RecHeader rh;
        rh.channel_id = RdU16(hdr + 4);    // 0..3 = magic "REC1"
        rh.kind = RdU16(hdr + 6);
        rh.seq = RdU32(hdr + 8);
        rh.stamp_ns = RdI64(hdr + 12);
        rh.recv_ns = RdI64(hdr + 20);
        rh.payload_size = RdU32(hdr + 28);
        rh.payload_crc32 = RdU32(hdr + 32);

        if (rh.payload_size > Components::kMaxPayloadSize) {
            error_ = "payload 长度异常（" + std::to_string(rh.payload_size) +
                     " B）@offset " + std::to_string(offset);
            return false;
        }
        if (expected != nullptr &&
            (rh.channel_id != expected->channel_id || rh.payload_size != expected->payload_size)) {
            error_ = "索引与记录不一致 @offset " + std::to_string(offset);
            return false;
        }
        const int64_t extra = static_cast<int64_t>(header_.rec_header_size) -
                              static_cast<int64_t>(Components::kRecHeaderSize);
        if (extra > 0) is_.seekg(extra, std::ios::cur);  // 未来格式：跳过未知字段

        payload.resize(rh.payload_size);
        if (rh.payload_size > 0) {
            is_.read(reinterpret_cast<char*>(payload.data()),
                     static_cast<std::streamsize>(rh.payload_size));
            if (is_.gcount() != static_cast<std::streamsize>(rh.payload_size)) {
                error_ = "payload 读取失败 @offset " + std::to_string(offset);
                return false;
            }
        }
        if (check_crc && Components::Crc32(payload.data(), payload.size()) != rh.payload_crc32) {
            error_ = "CRC 校验失败 @offset " + std::to_string(offset);
            return false;
        }

        info.header = rh;
        info.offset = offset;
        return true;
    }

    const Components::ChannelDesc* RecReader::channel(uint16_t id) const
    {
        for (const auto& ch : channels_) {
            if (ch.channel_id == id) return &ch;
        }
        return nullptr;
    }

}  // namespace RusRecorder::Storage
