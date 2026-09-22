#pragma once

// ════════════════════════════════════════════════════════════════════
//  记录写入器（storage：文件生命周期 + 记录流 + 尾索引）
//  ────────────────────────────────────────────────────────────────────
//  一个 RecWriter 管一个文件，生命周期：
//      Open(path)           创建/截断文件，记住头部字段（尚未落盘）
//      AddChannel(desc) × N 追加通道描述（须在 Start() 之前）
//      Start()              写 FileHeader + 通道表 → 记录流起点
//      WriteRecord(...) × N 追加记录（payload 可空）；内部维护通道内自增 seq
//      Close()              写尾索引 + FileFooter + 落盘并关闭
//
//  只有 Close() 成功，文件才算"带尾索引"；进程被强杀的文件里没有索引 / Footer，
//  但每条记录自带魔数与 CRC —— rus_sim_recorder_inspect --scan 仍能顺序扫描读回。
//
//  纯 std 实现（无 ROS 类型）：可离线单测、可被无 ROS 环境的工具复用。
//  线程模型：非线程安全 —— 必须由单一写线程独占调用。
// ════════════════════════════════════════════════════════════════════

#include <cstdint>
#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "components/bin_writer.hpp"
#include "components/rec_format.hpp"

namespace RusRecorder::Storage {

    /// `.rusrec` 单文件写入器
    class RecWriter {
    public:
        struct Options {
            std::string path;                 // 目标文件路径（父目录须已存在）
            size_t buffer_bytes = 1u << 20;   // 写缓冲（默认 1 MiB）
            int64_t created_unix_ns = 0;      // 录制开始时刻（0 = 由实现取当前时间）
        };

        struct Stats {
            uint64_t records = 0;        // 本文件已写记录数
            uint64_t payload_bytes = 0;  // payload 累计字节
        };

        /// 创建 / 截断文件（失败返回 false，error() 说明原因）
        bool Open(const Options& opt);

        /// 追加通道描述（仅可在 Open 之后、Start 之前调用）
        bool AddChannel(const Components::ChannelDesc& desc);

        /// 写文件头 + 通道表（此后才能写记录）
        bool Start();

        /// 追加一条记录（payload = nullptr / size = 0 表示空 payload）
        bool WriteRecord(uint16_t channel_id, int64_t stamp_ns, int64_t recv_ns,
                         const void* payload, size_t payload_size);

        /// 缓冲落盘（录制期间由上层按 flush 周期调用）
        bool Flush() { return writer_ ? writer_->Flush() : false; }

        /// 写尾索引 + Footer，落盘并关闭（可重复调用；未 Open 返回 false）
        bool Close();

        bool opened() const { return opened_; }
        bool started() const { return started_; }
        bool closed() const { return closed_; }
        uint64_t Offset() const { return writer_ ? writer_->Offset() : 0; }
        uint64_t index_offset() const { return index_offset_; }
        const Stats& stats() const { return stats_; }
        const std::string& path() const { return path_; }
        const std::string& error() const { return error_; }

    private:
        void set_error(const std::string& msg);

        std::ofstream os_;
        std::unique_ptr<Components::BinWriter> writer_;
        std::string path_;
        std::string error_;
        std::vector<Components::ChannelDesc> channels_;
        std::map<uint16_t, uint16_t> channel_kind_;   // channel_id → kind
        std::map<uint16_t, uint32_t> channel_seq_;    // channel_id → 下一个 seq
        std::vector<Components::IndexEntry> index_;
        Stats stats_;
        int64_t created_ns_ = 0;
        uint64_t index_offset_ = 0;
        bool opened_ = false;
        bool started_ = false;
        bool closed_ = false;
    };

}  // namespace RusRecorder::Storage
