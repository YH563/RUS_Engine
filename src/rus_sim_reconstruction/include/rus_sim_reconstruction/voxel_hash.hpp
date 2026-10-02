#pragma once

// ════════════════════════════════════════════════════════════════════
//  扁平体素哈希网格（reconstruction：体素 → 面元索引桶）
//  ────────────────────────────────────────────────────────────────────
//  开放寻址 + 线性探测 + 2 的幂容量，替代 std::unordered_map，省去
//  每个节点一次堆分配与红黑/链表的指针追逐。只插入、不删除（Clear 整体重置），
//  因此实现简单。
//
//  纯 std，无依赖。
// ════════════════════════════════════════════════════════════════════

#include <cstddef>
#include <cstdint>
#include <vector>

namespace RusReconstruction {

    struct VoxelKey {
        int32_t x = 0, y = 0, z = 0;
        bool operator==(const VoxelKey& o) const {
            return x == o.x && y == o.y && z == o.z;
        }
    };

    inline size_t VoxelKeyHash(const VoxelKey& k)
    {
        uint64_t h = 1469598103934665603ull;
        auto mix = [&h](int32_t v) { h ^= static_cast<uint32_t>(v); h *= 1099511628211ull; };
        mix(k.x); mix(k.y); mix(k.z);
        return static_cast<size_t>(h);
    }

    /// 体素桶：固定容量（max_surfels_per_voxel ≤ 4）
    class VoxelHash {
    public:
        static constexpr int kBucketCap = 4;

        struct Bucket {
            VoxelKey key;
            int32_t count = 0;
            int32_t idx[kBucketCap] = {-1, -1, -1, -1};
        };

        explicit VoxelHash(size_t initial_capacity = 1024)
        {
            allocate(initial_capacity);
        }

        void Clear()
        {
            std::fill(occupied_.begin(), occupied_.end(), uint8_t{0});
            size_ = 0;
        }

        size_t Size() const { return size_; }

        /// 查找体素桶（未找到返回 nullptr）
        const Bucket* Find(const VoxelKey& k) const
        {
            size_t i = VoxelKeyHash(k) & mask_;
            while (occupied_[i]) {
                if (slots_[i].key == k) return &slots_[i];
                i = (i + 1) & mask_;
            }
            return nullptr;
        }

        Bucket* Find(const VoxelKey& k)
        {
            size_t i = VoxelKeyHash(k) & mask_;
            while (occupied_[i]) {
                if (slots_[i].key == k) return &slots_[i];
                i = (i + 1) & mask_;
            }
            return nullptr;
        }

        /// 取或创建体素桶
        Bucket& GetOrCreate(const VoxelKey& k)
        {
            if ((size_ + 1) * 10 >= (mask_ + 1) * 7) grow();  // 装载因子 0.7
            size_t i = VoxelKeyHash(k) & mask_;
            while (occupied_[i]) {
                if (slots_[i].key == k) return slots_[i];
                i = (i + 1) & mask_;
            }
            occupied_[i] = 1;
            slots_[i].key = k;
            slots_[i].count = 0;
            ++size_;
            return slots_[i];
        }

        /// 遍历所有非空桶
        template <typename F>
        void ForEach(F&& f) const
        {
            for (size_t i = 0; i <= mask_; ++i) {
                if (occupied_[i]) f(slots_[i]);
            }
        }

    private:
        void allocate(size_t capacity)
        {
            size_t cap = 1024;
            while (cap < capacity) cap <<= 1;
            slots_.assign(cap, Bucket{});
            occupied_.assign(cap, uint8_t{0});
            mask_ = cap - 1;
            size_ = 0;
        }

        void grow()
        {
            std::vector<Bucket> old = std::move(slots_);
            std::vector<uint8_t> old_occ = std::move(occupied_);
            allocate((mask_ + 1) * 2);
            for (size_t i = 0; i < old.size(); ++i) {
                if (!old_occ[i]) continue;
                size_t j = VoxelKeyHash(old[i].key) & mask_;
                while (occupied_[j]) j = (j + 1) & mask_;
                occupied_[j] = 1;
                slots_[j] = old[i];
                ++size_;
            }
        }

        std::vector<Bucket> slots_;
        std::vector<uint8_t> occupied_;
        size_t mask_ = 0;
        size_t size_ = 0;
    };

}  // namespace RusReconstruction
