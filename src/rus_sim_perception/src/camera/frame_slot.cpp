#include "camera/frame_slot.hpp"

#include <utility>

namespace RusPerception::Camera {

    void FrameSlot::Push(CloudFrame frame)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        if (valid_) ++overwritten_;  // 上一帧还没被消费就被覆盖
        frame_ = std::move(frame);
        valid_ = true;
        ++pushed_;
    }

    bool FrameSlot::TakeNewerThan(double since, CloudFrame& out)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        if (!valid_) return false;

        valid_ = false;
        if (frame_.stamp <= since) {
            ++stale_;  // 同一帧（或更旧）已处理过：丢弃，防止重复入图
            return false;
        }
        out = std::move(frame_);
        frame_ = CloudFrame{};
        return true;
    }

    bool FrameSlot::Empty() const
    {
        std::lock_guard<std::mutex> lock(mtx_);
        return !valid_;
    }

    void FrameSlot::Clear()
    {
        std::lock_guard<std::mutex> lock(mtx_);
        frame_ = CloudFrame{};
        valid_ = false;
    }

    uint64_t FrameSlot::Pushed() const
    {
        std::lock_guard<std::mutex> lock(mtx_);
        return pushed_;
    }

    uint64_t FrameSlot::Overwritten() const
    {
        std::lock_guard<std::mutex> lock(mtx_);
        return overwritten_;
    }

    uint64_t FrameSlot::Stale() const
    {
        std::lock_guard<std::mutex> lock(mtx_);
        return stale_;
    }

}  // namespace RusPerception::Camera
