#include "rus_sim_data/data_recorder.hpp"

#include <cstring>
#include <cstddef>

namespace RusSimData {

    // ============================================================
    //  录制
    // ============================================================

    bool DataRecorder::StartRecording(const std::string& path) {
        StopRecording();  // 确保上一个录制已结束

        std::string out_path = path;
        if (out_path.empty()) {
            auto now = std::chrono::system_clock::now();
            auto t = std::chrono::system_clock::to_time_t(now);
            char buf[64];
            std::strftime(buf, sizeof(buf), "recording_%Y%m%d_%H%M%S.bin", std::localtime(&t));
            out_path = buf;
        }

        file_.open(out_path, std::ios::binary);
        if (!file_) return false;

        // 写占位 Header，关闭时再更新 frame_count
        FileHeader header{};
        header.magic       = kMagic;
        header.version     = kVersion;
        header.num_joints  = 0;
        header.frame_count = 0;
        file_.write(reinterpret_cast<const char*>(&header), sizeof(header));

        num_joints_     = 0;
        written_frames_ = 0;
        stop_writer_    = false;

        // 启动写线程
        writer_thread_ = std::thread(&DataRecorder::writer_thread_func, this);

        return true;
    }

    void DataRecorder::RecordFrame(const RusUtils::RobotState& state) {
        if (stop_writer_) return;

        // 首帧提取关节数（同步取，后续写线程用）
        if (written_frames_ == 0 && write_queue_.empty()) {
            num_joints_ = static_cast<uint32_t>(state.joint_pos.size());
        }
        std::lock_guard<std::mutex> lock(queue_mtx_);
        write_queue_.push_back(state);
        queue_cv_.notify_one();
    }

    void DataRecorder::StopRecording() {
        stop_writer_ = true;
        queue_cv_.notify_one();
        if (writer_thread_.joinable())
            writer_thread_.join();

        if (file_.is_open()) {
            // 更新 frame_count
            file_.seekp(offsetof(FileHeader, frame_count));
            file_.write(reinterpret_cast<const char*>(&written_frames_),
                        sizeof(written_frames_));
            file_.close();
        }
    }

    // ---- writer_thread_func — 后台写线程 ----
    void DataRecorder::writer_thread_func() {
        bool header_updated = false;

        while (true) {
            std::unique_lock<std::mutex> lock(queue_mtx_);
            queue_cv_.wait(lock, [this] {
                return !write_queue_.empty() || stop_writer_;
            });

            if (stop_writer_ && write_queue_.empty())
                break;

            RusUtils::RobotState frame = std::move(write_queue_.front());
            write_queue_.pop_front();
            lock.unlock();  // 释放队列锁，后续文件 I/O 不需要它

            // 首帧：回写 num_joints 到文件头
            if (!header_updated && num_joints_ > 0) {
                file_.seekp(offsetof(FileHeader, num_joints));
                file_.write(reinterpret_cast<const char*>(&num_joints_),
                            sizeof(num_joints_));
                file_.seekp(0, std::ios::end);
                header_updated = true;
            }

            write_frame(frame);
            ++written_frames_;
        }
    }

    // ---- write_frame — 写入单帧到文件 ----
    void DataRecorder::write_frame(const RusUtils::RobotState& state) {
        file_.write(reinterpret_cast<const char*>(&state.timestamp), sizeof(double));
        file_.write(reinterpret_cast<const char*>(state.joint_pos.data()),
                    state.joint_pos.size() * sizeof(double));
        file_.write(reinterpret_cast<const char*>(state.joint_vel.data()),
                    state.joint_vel.size() * sizeof(double));
        file_.write(reinterpret_cast<const char*>(state.joint_acc.data()),
                    state.joint_acc.size() * sizeof(double));
        file_.write(reinterpret_cast<const char*>(state.effort.data()),
                    state.effort.size() * sizeof(double));
        file_.write(reinterpret_cast<const char*>(state.flange_pos.data()),
                    state.flange_pos.size() * sizeof(double));
    }

    // ============================================================
    //  回放
    // ============================================================

    bool DataRecorder::LoadRecording(const std::string& path) {
        Clear();

        std::ifstream in(path, std::ios::binary);
        if (!in) return false;

        FileHeader header;
        in.read(reinterpret_cast<char*>(&header), sizeof(header));
        if (!in || header.magic != kMagic || header.version != kVersion)
            return false;

        uint32_t nj = header.num_joints;
        uint32_t nf = header.frame_count;
        if (nj == 0 || nf == 0) return true;

        frames_.reserve(nf);

        for (uint32_t i = 0; i < nf; ++i) {
            RusUtils::RobotState state;
            state.joint_pos .resize(nj);
            state.joint_vel .resize(nj);
            state.joint_acc .resize(nj);
            state.effort   .resize(nj);
            state.flange_pos.resize(6);

            in.read(reinterpret_cast<char*>(&state.timestamp), sizeof(double));
            in.read(reinterpret_cast<char*>(state.joint_pos.data()), nj * sizeof(double));
            in.read(reinterpret_cast<char*>(state.joint_vel.data()), nj * sizeof(double));
            in.read(reinterpret_cast<char*>(state.joint_acc.data()), nj * sizeof(double));
            in.read(reinterpret_cast<char*>(state.effort.data()),     nj * sizeof(double));
            in.read(reinterpret_cast<char*>(state.flange_pos.data()), 6  * sizeof(double));

            if (!in) return false;
            frames_.push_back(std::move(state));
        }

        num_joints_ = nj;
        return true;
    }

    bool DataRecorder::GetFrame(size_t index, RusUtils::RobotState& state) const {
        if (index >= frames_.size()) return false;
        state = frames_[index];
        return true;
    }

    void DataRecorder::Clear() {
        StopRecording();
        frames_.clear();
        num_joints_     = 0;
        written_frames_ = 0;
    }

}  // namespace RusSimData
