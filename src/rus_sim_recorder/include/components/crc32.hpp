#pragma once

// ════════════════════════════════════════════════════════════════════
//  CRC32 校验（components：纯工具，无任何依赖）
//  ────────────────────────────────────────────────────────────────────
//  IEEE 802.3（反射多项式 0xEDB88320，初值 / 终值异或 0xFFFFFFFF），
//  与 zlib / gzip / PNG 同一算法。记录文件用它逐条校验 payload 完整性：
//  磁盘坏块 / 写入截断当场就能发现，而不是等到离线回放时解出一堆垃圾。
//
//  查表在编译期生成（constexpr），运行时零初始化开销。
// ════════════════════════════════════════════════════════════════════

#include <cstddef>
#include <cstdint>

namespace RusRecorder::Components {

    namespace detail {
        /// CRC32 查表（编译期生成）
        struct Crc32Table {
            uint32_t v[256];
            constexpr Crc32Table() : v{}
            {
                for (uint32_t i = 0; i < 256; ++i) {
                    uint32_t c = i;
                    for (int k = 0; k < 8; ++k) {
                        c = (c & 1u) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
                    }
                    v[i] = c;
                }
            }
        };
        inline constexpr Crc32Table kCrc32Table{};
    }  // namespace detail

    /// 一次性计算 CRC32（data=nullptr / len=0 → 返回 0）
    inline uint32_t Crc32(const void* data, size_t len)
    {
        const auto* p = static_cast<const uint8_t*>(data);
        uint32_t crc = 0xFFFFFFFFu;
        for (size_t i = 0; i < len; ++i) {
            crc = detail::kCrc32Table.v[(crc ^ p[i]) & 0xFFu] ^ (crc >> 8);
        }
        return crc ^ 0xFFFFFFFFu;
    }

    /// 增量 CRC32（分段喂数据：大 payload 边收边算，不必先攒成全块）
    class Crc32Stream {
    public:
        void Update(const void* data, size_t len)
        {
            const auto* p = static_cast<const uint8_t*>(data);
            for (size_t i = 0; i < len; ++i) {
                state_ = detail::kCrc32Table.v[(state_ ^ p[i]) & 0xFFu] ^ (state_ >> 8);
            }
        }

        uint32_t Value() const { return state_ ^ 0xFFFFFFFFu; }

    private:
        uint32_t state_ = 0xFFFFFFFFu;
    };

}  // namespace RusRecorder::Components
