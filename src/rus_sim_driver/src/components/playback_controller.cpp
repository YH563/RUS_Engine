#include "components/playback_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <cstddef>

namespace RusRobotDriver {

// ============================================================
//  数据源
// ============================================================

void PlaybackController::SetFrames(
    const std::vector<RusRobotDriver::RobotState>* frames)
{
    frames_ = frames;
    Reset();
    if (frames_ && !frames_->empty()) {
        total_time_ = frames_->back().timestamp - frames_->front().timestamp;
    } else {
        total_time_ = 0.0;
    }
}

// ============================================================
//  播放控制
// ============================================================

void PlaybackController::Play() {
    if (!frames_ || frames_->empty()) return;
    if (current_frame_ >= frames_->size() - 1) {
        current_frame_ = 0;
        playback_time_ = 0.0;
    }
    playing_ = true;
}

void PlaybackController::Pause() {
    playing_ = false;
}

void PlaybackController::Stop() {
    playing_ = false;
    current_frame_ = 0;
    playback_time_ = 0.0;
}

// ============================================================
//  速度
// ============================================================

void PlaybackController::SetSpeed(double speed) {
    speed_ = std::clamp(speed, 0.01, 100.0);
}

// ============================================================
//  跳转
// ============================================================

bool PlaybackController::SeekToFrame(size_t index) {
    if (!frames_ || frames_->empty()) return false;
    if (index >= frames_->size()) return false;
    current_frame_ = index;
    playback_time_ = frames_->at(index).timestamp - frames_->front().timestamp;
    return true;
}

bool PlaybackController::SeekToTime(double seconds) {
    if (!frames_ || frames_->empty()) return false;
    if (seconds <= 0.0) { SeekToStart(); return true; }
    if (seconds >= total_time_) { SeekToEnd(); return true; }

    playback_time_ = seconds;
    sync_frame();
    return true;
}

void PlaybackController::SeekForward(double seconds) {
    SeekToTime(playback_time_ + seconds);
}

void PlaybackController::SeekBackward(double seconds) {
    SeekToTime(playback_time_ - seconds);
}

void PlaybackController::SeekToStart() {
    current_frame_ = 0;
    playback_time_ = 0.0;
}

void PlaybackController::SeekToEnd() {
    if (!frames_ || frames_->empty()) return;
    current_frame_ = frames_->size() - 1;
    playback_time_ = total_time_;
}

bool PlaybackController::SeekToProgress(double ratio) {
    if (!frames_ || frames_->empty()) return false;
    ratio = std::clamp(ratio, 0.0, 1.0);
    return SeekToTime(ratio * total_time_);
}

// ============================================================
//  步进
// ============================================================

bool PlaybackController::StepForward() {
    if (!frames_ || frames_->empty()) return false;
    if (current_frame_ + 1 < frames_->size()) {
        ++current_frame_;
        playback_time_ = frames_->at(current_frame_).timestamp
                        - frames_->front().timestamp;
        return true;
    }
    if (loop_ && !frames_->empty()) {
        current_frame_ = 0;
        playback_time_ = 0.0;
        return true;
    }
    return false;
}

bool PlaybackController::StepBackward() {
    if (!frames_ || frames_->empty()) return false;
    if (current_frame_ > 0) {
        --current_frame_;
        playback_time_ = frames_->at(current_frame_).timestamp
                        - frames_->front().timestamp;
        return true;
    }
    if (loop_ && !frames_->empty()) {
        current_frame_ = frames_->size() - 1;
        playback_time_ = total_time_;
        return true;
    }
    return false;
}

// ============================================================
//  查询
// ============================================================

double PlaybackController::GetProgress() const {
    if (total_time_ <= 0.0) return 0.0;
    return std::clamp(playback_time_ / total_time_, 0.0, 1.0);
}

// ============================================================
//  主循环更新
// ============================================================

bool PlaybackController::Update(double dt_seconds,
                                 RusRobotDriver::RobotState& out_state,
                                 bool* frame_changed)
{
    if (!frames_ || frames_->empty()) return false;

    bool changed = false;
    size_t prev_frame = current_frame_;

    if (playing_ && total_time_ > 0.0) {
        playback_time_ += dt_seconds * speed_;

        if (playback_time_ >= total_time_) {
            if (loop_) {
                playback_time_ = std::fmod(playback_time_, total_time_);
            } else {
                playback_time_ = total_time_;
                playing_ = false;
            }
        }

        sync_frame();
        changed = (current_frame_ != prev_frame);
    }

    // 帧间线性插值
    if (interpolate_ && current_frame_ + 1 < frames_->size()) {
        const auto& cur  = frames_->at(current_frame_);
        const auto& next = frames_->at(current_frame_ + 1);
        double t0 = cur.timestamp  - frames_->front().timestamp;
        double t1 = next.timestamp - frames_->front().timestamp;
        double dur = t1 - t0;

        if (dur > 0.0) {
            double t = (playback_time_ - t0) / dur;
            t = std::clamp(t, 0.0, 1.0);

            if (t > 0.0 && t < 1.0) {
                lerp_state(cur, next, t, out_state);
                if (frame_changed) *frame_changed = changed;
                return true;
            }
        }
    }

    out_state = frames_->at(current_frame_);
    if (frame_changed) *frame_changed = changed;
    return true;
}

bool PlaybackController::GetCurrentState(
    RusRobotDriver::RobotState& state) const
{
    if (!frames_ || frames_->empty()) return false;
    state = frames_->at(current_frame_);
    return true;
}

void PlaybackController::Reset() {
    playing_ = false;
    current_frame_ = 0;
    playback_time_ = 0.0;
    speed_ = 1.0;
    loop_ = true;
    interpolate_ = true;
}

// ============================================================
//  序列化
// ============================================================

std::vector<uint8_t> PlaybackController::SerializeFrames() const {
    std::vector<uint8_t> blob;
    if (!frames_ || frames_->empty()) return blob;

    uint32_t nj = static_cast<uint32_t>(frames_->front().joint_pos.size());
    uint32_t nf = static_cast<uint32_t>(frames_->size());
    size_t frame_bytes = sizeof(double)                  // timestamp
                       + nj * sizeof(double) * 4         // pos+vel+acc+effort
                       + 6 * sizeof(double);             // flange_pos

    size_t header_size = 3 * sizeof(uint32_t);           // magic + nj + nf
    blob.resize(header_size + nf * frame_bytes);

    // 写入小头
    uint8_t* ptr = blob.data();
    auto write32 = [&](uint32_t v) {
        std::memcpy(ptr, &v, sizeof(v)); ptr += sizeof(v);
    };
    write32(0x52534452u);  // magic "RSDR"
    write32(nj);
    write32(nf);

    for (const auto& f : *frames_) {
        std::memcpy(ptr, &f.timestamp, sizeof(double));
        ptr += sizeof(double);
        std::memcpy(ptr, f.joint_pos.data(),  nj * sizeof(double)); ptr += nj * sizeof(double);
        std::memcpy(ptr, f.joint_vel.data(),  nj * sizeof(double)); ptr += nj * sizeof(double);
        std::memcpy(ptr, f.joint_acc.data(),  nj * sizeof(double)); ptr += nj * sizeof(double);
        std::memcpy(ptr, f.effort.data(),     nj * sizeof(double)); ptr += nj * sizeof(double);
        std::memcpy(ptr, f.flange_pos.data(), 6  * sizeof(double)); ptr += 6  * sizeof(double);
    }

    return blob;
}

std::vector<RusRobotDriver::RobotState>
PlaybackController::DeserializeFrames(const uint8_t* data, size_t size,
                                       uint32_t num_joints)
{
    std::vector<RusRobotDriver::RobotState> frames;
    if (!data || size < 3 * sizeof(uint32_t)) return frames;

    const uint8_t* ptr = data;
    auto read32 = [&]() -> uint32_t {
        uint32_t v;
        std::memcpy(&v, ptr, sizeof(v)); ptr += sizeof(v);
        return v;
    };

    uint32_t magic = read32();
    uint32_t nj    = read32();
    uint32_t nf    = read32();

    if (magic != 0x52534452u) return frames;
    if (num_joints > 0) nj = num_joints;  // 调用方指定优先

    size_t frame_bytes = sizeof(double)
                       + nj * sizeof(double) * 4
                       + 6 * sizeof(double);
    if (ptr + nf * frame_bytes > data + size) return frames;

    frames.reserve(nf);
    for (uint32_t i = 0; i < nf; ++i) {
        RusRobotDriver::RobotState st;
        st.joint_pos .resize(nj);
        st.joint_vel .resize(nj);
        st.joint_acc .resize(nj);
        st.effort   .resize(nj);
        st.flange_pos .resize(6);

        std::memcpy(&st.timestamp,   ptr, sizeof(double));           ptr += sizeof(double);
        std::memcpy(st.joint_pos.data(),  ptr, nj * sizeof(double)); ptr += nj * sizeof(double);
        std::memcpy(st.joint_vel.data(),  ptr, nj * sizeof(double)); ptr += nj * sizeof(double);
        std::memcpy(st.joint_acc.data(),  ptr, nj * sizeof(double)); ptr += nj * sizeof(double);
        std::memcpy(st.effort.data(),     ptr, nj * sizeof(double)); ptr += nj * sizeof(double);
        std::memcpy(st.flange_pos.data(), ptr, 6  * sizeof(double)); ptr += 6  * sizeof(double);

        frames.push_back(std::move(st));
    }

    return frames;
}

// ============================================================
//  内部方法
// ============================================================

void PlaybackController::sync_frame() {
    if (!frames_ || frames_->empty()) return;

    double target = playback_time_ + frames_->front().timestamp;

    // upper_bound: 第一个 timestamp > target 的帧
    auto it = std::upper_bound(
        frames_->begin(), frames_->end(), target,
        [](double t, const RusRobotDriver::RobotState& s) {
            return t < s.timestamp;
        });

    if (it == frames_->begin()) {
        current_frame_ = 0;
    } else {
        current_frame_ = static_cast<size_t>(
            std::distance(frames_->begin(), it - 1));
    }
}

void PlaybackController::lerp_state(const RusRobotDriver::RobotState& a,
                                     const RusRobotDriver::RobotState& b,
                                     double t,
                                     RusRobotDriver::RobotState& out) {
    double s = 1.0 - t;
    out.flange_pos = a.flange_pos * s + b.flange_pos * t;
    out.joint_pos  = a.joint_pos  * s + b.joint_pos  * t;
    out.joint_vel  = a.joint_vel  * s + b.joint_vel  * t;
    out.joint_acc  = a.joint_acc  * s + b.joint_acc  * t;
    out.effort     = a.effort     * s + b.effort     * t;
    out.timestamp  = a.timestamp  * s + b.timestamp  * t;
}

}  // namespace RusRobotDriver
