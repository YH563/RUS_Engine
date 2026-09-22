#pragma once

// ════════════════════════════════════════════════════════════════════
//  小端缓冲写入器（components：二进制落盘基元）
//  ────────────────────────────────────────────────────────────────────
//  - 所有定长字段显式按小端写入（不依赖主机字节序 / 结构体 padding）；
//  - 先在内存缓冲攒够再落盘（`Flush()`）：录制热点路径上少系统调用；
//  - 任一写失败即置位错误，之后所有写入变成空操作 —— 上层据此停录并报错，
//    避免继续往坏文件里堆数据（或写出长度对不上的记录）。
// ════════════════════════════════════════════════════════════════════

#include <cstddef>
#include <cstdint>
#include <ostream>
#include <string>
#include <vector>

namespace RusRecorder::Components {

    /// 小端 + 缓冲的二进制写入器
    class BinWriter {
    public:
        explicit BinWriter(std::ostream& os, size_t buffer_bytes = 1u << 20)
            : os_(os), buf_(buffer_bytes)
        {
            buf_.clear();
        }

        void U8(uint8_t v) { put(&v, 1); }

        void U16(uint16_t v)
        {
            const uint8_t b[2] = {static_cast<uint8_t>(v & 0xFFu),
                                  static_cast<uint8_t>((v >> 8) & 0xFFu)};
            put(b, 2);
        }

        void U32(uint32_t v)
        {
            const uint8_t b[4] = {static_cast<uint8_t>(v & 0xFFu),
                                  static_cast<uint8_t>((v >> 8) & 0xFFu),
                                  static_cast<uint8_t>((v >> 16) & 0xFFu),
                                  static_cast<uint8_t>((v >> 24) & 0xFFu)};
            put(b, 4);
        }

        void U64(uint64_t v)
        {
            uint8_t b[8];
            for (int i = 0; i < 8; ++i) b[i] = static_cast<uint8_t>((v >> (8 * i)) & 0xFFu);
            put(b, 8);
        }

        void I64(int64_t v) { U64(static_cast<uint64_t>(v)); }

        void Bytes(const void* data, size_t len) { put(data, len); }

        /// u16 长度前缀 + UTF-8 字节（文件中所有字符串字段的编码）
        void Str(const std::string& s)
        {
            U16(static_cast<uint16_t>(s.size()));
            if (!s.empty()) Bytes(s.data(), s.size());
        }

        /// 当前逻辑写偏移（含尚未落盘的缓冲字节）
        uint64_t Offset() const { return offset_; }

        /// 缓冲中未落盘的字节数
        size_t PendingBytes() const { return buf_.size(); }

        /// 缓冲落盘（写失败 → ok() 变 false）
        bool Flush()
        {
            if (!ok_) return false;
            if (!buf_.empty()) {
                os_.write(reinterpret_cast<const char*>(buf_.data()),
                          static_cast<std::streamsize>(buf_.size()));
                if (!os_) {
                    ok_ = false;
                    error_ = "写盘失败（磁盘满 / 设备错误？）";
                }
                buf_.clear();
            }
            os_.flush();
            if (!os_) {
                ok_ = false;
                error_ = "flush 失败（磁盘满 / 设备错误？）";
            }
            return ok_;
        }

        bool ok() const { return ok_; }
        const std::string& error() const { return error_; }

    private:
        /// 落一块字节：大块直写（绕过缓冲），小块进缓冲（攒满则先 Flush）
        void put(const void* data, size_t len)
        {
            if (!ok_ || len == 0) return;
            const auto* b = static_cast<const uint8_t*>(data);

            if (len >= buf_.capacity()) {           // 超大块：先清缓冲，再直写
                if (!Flush()) return;
                os_.write(reinterpret_cast<const char*>(b), static_cast<std::streamsize>(len));
                if (!os_) {
                    ok_ = false;
                    error_ = "写盘失败（磁盘满 / 设备错误？）";
                    return;
                }
                offset_ += len;
                return;
            }
            if (buf_.size() + len > buf_.capacity()) {
                if (!Flush()) return;
            }
            buf_.insert(buf_.end(), b, b + len);
            offset_ += len;
        }

        std::ostream& os_;
        std::vector<uint8_t> buf_;
        uint64_t offset_ = 0;   // 逻辑偏移（缓冲 + 已落盘）
        bool ok_ = true;
        std::string error_;
    };

}  // namespace RusRecorder::Components
