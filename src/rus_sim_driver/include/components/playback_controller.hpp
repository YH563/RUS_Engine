#pragma once

#include <vector>
#include <cstddef>
#include <cstdint>

#include "components/types.hpp"

namespace RusRobotDriver {

    /**
    * @brief 回放控制器——像播放视频一样控制运动数据回放
    *
    * 配合 DataRecorder 使用：DataRecorder::LoadRecording() 加载数据后，
    * 将 frames 指针传给 PlaybackController。
    *
    * 支持：
    *  - 播放 / 暂停
    *  - 速度调节 (0.01x ~ 100.0x)
    *  - 按帧 / 按时间 / 按进度跳转
    *  - 逐帧步进
    *  - 循环播放
    *  - 帧间线性插值（平滑回放）
    *
    * 典型用法：
    * @code
    *   PlaybackController player;
    *   player.SetFrames(&frames);
    *   player.Play();
    *   player.SetSpeed(2.0);
    *
    *   while (running) {
    *       RobotState state;
    *       player.Update(0.01, state);  // dt = 10ms
    *       apply(state);
    *   }
    * @endcode
    */
    class PlaybackController {
    public:
        PlaybackController() = default;

        /// ── 数据源 ──
        void SetFrames(const std::vector<RusRobotDriver::RobotState>* frames);
        const std::vector<RusRobotDriver::RobotState>* GetFrames() const { return frames_; }

        /// ── 播放控制 ──
        void Play();
        void Pause();
        void Stop();

        bool IsPlaying() const { return playing_; }
        bool IsPaused()  const { return !playing_ && current_frame_ > 0; }
        bool IsStopped()  const { return !playing_ && current_frame_ == 0; }

        /// ── 速度 ──
        void  SetSpeed(double speed);
        double GetSpeed() const { return speed_; }

        /// ── 跳转 ──
        bool SeekToFrame(size_t index);
        bool SeekToTime(double seconds);
        void SeekForward(double seconds);
        void SeekBackward(double seconds);
        void SeekToStart();
        void SeekToEnd();
        bool SeekToProgress(double ratio);

        /// ── 步进 ──
        bool StepForward();
        bool StepBackward();

        /// ── 循环 ──
        void  SetLoop(bool enable) { loop_ = enable; }
        bool  GetLoop() const { return loop_; }

        /// ── 插值 ──
        void  SetInterpolation(bool enable) { interpolate_ = enable; }
        bool  GetInterpolation() const { return interpolate_; }

        /// ── 查询 ──
        size_t GetCurrentFrame() const { return current_frame_; }
        double GetCurrentTime()  const { return playback_time_; }
        double GetTotalTime()    const { return total_time_; }
        size_t GetTotalFrames()  const { return frames_ ? frames_->size() : 0; }
        double GetProgress()     const;

        /// ── 主循环更新 ──
        /**
        * @brief 按时间增量推进播放
        * @param dt_seconds  距离上次调用的时间增量（秒）
        * @param[out] out_state 输出的当前帧状态
        * @param[out] frame_changed 是否切换到了新帧（可选）
        * @return true 数据有效
        */
        bool Update(double dt_seconds,
                    RusRobotDriver::RobotState& out_state,
                    bool* frame_changed = nullptr);

        bool GetCurrentState(RusRobotDriver::RobotState& state) const;

        /// ── 重置 ──
        void Reset();

        /// ── 序列化 ──
        /** 将所有帧打包为连续内存块（用于网络传输等场景） */
        std::vector<uint8_t> SerializeFrames() const;

        /** 从连续内存块反序列化帧 */
        static std::vector<RusRobotDriver::RobotState> DeserializeFrames(
            const uint8_t* data, size_t size, uint32_t num_joints);

    private:
        void sync_frame();
        static void lerp_state(const RusRobotDriver::RobotState& a,
                            const RusRobotDriver::RobotState& b,
                            double t,
                            RusRobotDriver::RobotState& out);

        const std::vector<RusRobotDriver::RobotState>* frames_ = nullptr;

        size_t current_frame_ = 0;
        double playback_time_ = 0.0;
        double speed_         = 1.0;
        double total_time_    = 0.0;

        bool playing_     = false;
        bool loop_        = true;
        bool interpolate_ = true;
    };

}  // namespace RusRobotDriver
