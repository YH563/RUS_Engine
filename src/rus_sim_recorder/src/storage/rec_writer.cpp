#include "storage/rec_writer.hpp"

#include <chrono>
#include <string>
#include <utility>

#include "components/crc32.hpp"

namespace RusRecorder::Storage {

    namespace {
        /// 本机系统时钟（纳秒）：记录"何时收到"，与消息时间戳（ROS 时间）区分
        int64_t NowUnixNs()
        {
            return std::chrono::duration_cast<std::chrono::nanoseconds>(
                       std::chrono::system_clock::now().time_since_epoch())
                .count();
        }
    }  // namespace

    void RecWriter::set_error(const std::string& msg) { error_ = msg; }

    bool RecWriter::Open(const Options& opt)
    {
        if (opened_) {
            set_error("已打开（请先 Close）");
            return false;
        }
        if (opt.path.empty()) {
            set_error("文件路径为空");
            return false;
        }

        path_ = opt.path;
        created_ns_ = opt.created_unix_ns > 0 ? opt.created_unix_ns : NowUnixNs();

        os_.open(path_, std::ios::binary | std::ios::trunc);
        if (!os_.is_open()) {
            set_error("无法创建文件：" + path_);
            return false;
        }

        writer_ = std::make_unique<Components::BinWriter>(os_, opt.buffer_bytes);
        channels_.clear();
        channel_kind_.clear();
        channel_seq_.clear();
        index_.clear();
        stats_ = Stats{};
        error_.clear();
        index_offset_ = 0;
        opened_ = true;
        started_ = false;
        closed_ = false;
        return true;
    }

    bool RecWriter::AddChannel(const Components::ChannelDesc& desc)
    {
        if (!opened_ || started_) {
            set_error("AddChannel 只能在 Open 之后、Start 之前调用");
            return false;
        }
        if (channels_.size() >= 0xFFFFu) {
            set_error("通道数超上限（65535）");
            return false;
        }
        if (channel_kind_.count(desc.channel_id) != 0) {
            set_error("通道 id 重复：" + std::to_string(desc.channel_id));
            return false;
        }
        channels_.push_back(desc);
        channel_kind_[desc.channel_id] = desc.kind;
        channel_seq_[desc.channel_id] = 0;
        return true;
    }

    bool RecWriter::Start()
    {
        if (!opened_ || started_) {
            set_error("Start 状态非法（未 Open 或已 Start）");
            return false;
        }

        // ── FileHeader（字段顺序与 rec_format.hpp 的分解式一致）──
        writer_->Bytes(Components::kFileMagic, sizeof(Components::kFileMagic));
        writer_->U16(Components::kFormatVersion);
        writer_->U16(Components::kFileHeaderSize);
        writer_->U16(Components::kRecHeaderSize);
        writer_->U16(Components::kIndexEntrySize);
        writer_->U32(0);  // flags（预留）
        writer_->I64(created_ns_);
        writer_->U16(static_cast<uint16_t>(channels_.size()));
        writer_->U16(0);  // reserved
        for (int i = 0; i < 32; ++i) writer_->U8(0);  // reserved2（未来扩展位）

        // ── 通道表 ──
        for (const auto& ch : channels_) {
            writer_->U16(ch.channel_id);
            writer_->U16(ch.kind);
            writer_->Str(ch.topic);
            writer_->Str(ch.type_name);
            writer_->Str(ch.note);
        }

        if (!writer_->ok()) {
            set_error("写文件头失败：" + writer_->error());
            return false;
        }
        started_ = true;
        return true;
    }

    bool RecWriter::WriteRecord(uint16_t channel_id, int64_t stamp_ns, int64_t recv_ns,
                                const void* payload, size_t payload_size)
    {
        if (!started_ || closed_) {
            set_error("未 Start 或已 Close");
            return false;
        }
        if (payload_size > Components::kMaxPayloadSize) {
            set_error("payload 超上限");
            return false;
        }
        const auto kind_it = channel_kind_.find(channel_id);
        if (kind_it == channel_kind_.end()) {
            set_error("未声明的通道 id：" + std::to_string(channel_id));
            return false;
        }
        if (payload_size > 0 && payload == nullptr) {
            set_error("payload 指针为空但长度非 0");
            return false;
        }

        const uint32_t crc = Components::Crc32(payload, payload_size);
        const uint64_t offset = writer_->Offset();
        const uint32_t seq = channel_seq_[channel_id]++;

        Components::IndexEntry entry;
        entry.offset = static_cast<int64_t>(offset);
        entry.stamp_ns = stamp_ns;
        entry.channel_id = channel_id;
        entry.kind = kind_it->second;
        entry.payload_size = static_cast<uint32_t>(payload_size);
        entry.payload_crc32 = crc;
        index_.push_back(entry);

        writer_->Bytes(Components::kRecMagic, sizeof(Components::kRecMagic));
        writer_->U16(channel_id);
        writer_->U16(kind_it->second);
        writer_->U32(seq);
        writer_->I64(stamp_ns);
        writer_->I64(recv_ns);
        writer_->U32(static_cast<uint32_t>(payload_size));
        writer_->U32(crc);
        writer_->U32(0);  // reserved
        writer_->Bytes(payload, payload_size);

        if (!writer_->ok()) {
            set_error("写记录失败：" + writer_->error());
            return false;
        }
        stats_.records++;
        stats_.payload_bytes += payload_size;
        return true;
    }

    bool RecWriter::Close()
    {
        if (!opened_) {
            set_error("未打开");
            return false;
        }
        if (closed_) return true;

        if (started_) {
            index_offset_ = writer_->Offset();
            for (const auto& e : index_) {
                writer_->I64(e.offset);
                writer_->I64(e.stamp_ns);
                writer_->U16(e.channel_id);
                writer_->U16(e.kind);
                writer_->U32(e.payload_size);
                writer_->U32(e.payload_crc32);
                writer_->U32(0);  // reserved
            }
            const uint64_t file_size = writer_->Offset() + Components::kFooterSize;
            writer_->I64(static_cast<int64_t>(index_offset_));
            writer_->I64(static_cast<int64_t>(index_.size()));
            writer_->I64(static_cast<int64_t>(file_size));
            writer_->Bytes(Components::kFooterMagic, sizeof(Components::kFooterMagic));
        }

        const bool ok = writer_->Flush();
        const std::string write_err = ok ? std::string() : writer_->error();
        writer_.reset();
        os_.close();
        closed_ = true;
        opened_ = false;   // 允许复用同一 RecWriter 打开下一个文件（滚动录制）
        if (!ok) {
            set_error("关闭前落盘失败：" + write_err);
            return false;
        }
        return true;
    }

}  // namespace RusRecorder::Storage
