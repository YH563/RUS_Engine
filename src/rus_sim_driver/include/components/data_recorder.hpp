#pragma once

#include <string>
#include <fstream>
#include <vector>
#include <deque>
#include <cstdint>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>

#include "components/types.hpp"

namespace RusRobotDriver {

    /**
    * @brief 运动数据记录器
    *
    * 录制 RobotState 帧到二进制文件，支持加载回放。
    * 职责单一：只做运动数据的录制与回放，与文本日志分离。
    *
    * 文件格式：
    *   [Header]  magic(4B) + version(4B) + num_joints(4B) + frame_count(4B)
    *   [Frames]  timestamp(8B) + joint_pos(num_joints*8B) + joint_vel(num_joints*8B)
    *            + joint_acc(num_joints*8B) + effort(num_joints*8B) + flange_pos(6*8B)
    */
    class DataRecorder {
    public:
        DataRecorder() = default;
        ~DataRecorder() { StopRecording(); }

        /** ── 录制 ── */

        /**
        * @brief 开始录制到文件
        * @param path 文件路径
        * @return true 打开成功
        */
        bool StartRecording(const std::string& path);

        /**
        * @brief 录制一帧状态
        */
        void RecordFrame(const RusRobotDriver::RobotState& state);

        /**
        * @brief 停止录制，关闭文件
        */
        void StopRecording();

        /**
        * @brief 是否正在录制
        */
        bool IsRecording() const { return !stop_writer_; }

        /** ── 回放 ── */

        /**
        * @brief 从文件加载录制数据（用于回放）
        *
        * @param path 文件路径
        * @return true 加载成功
        */
        bool LoadRecording(const std::string& path);

        /**
        * @brief 获取录制的总帧数
        */
        size_t GetFrameCount() const { return frames_.size(); }

        /**
        * @brief 获取指定帧（用于回放）
        * @param index 帧索引 [0, GetFrameCount())
        * @param [out] state 输出的状态
        * @return true 索引有效
        */
        bool GetFrame(size_t index, RusRobotDriver::RobotState& state) const;

        /**
        * @brief 清除已加载的录制数据
        */
        void Clear();

    private:
        /** 二进制文件头 */
        struct FileHeader {
            uint32_t magic;        /**< 魔术字 "RSDR" */
            uint32_t version;      /**< 版本号 */
            uint32_t num_joints;   /**< 关节数 */
            uint32_t frame_count;  /**< 总帧数 */
        };

        /** 写线程入口 */
        void writer_thread_func();

        /** 将一帧写入已打开的文件 */
        void write_frame(const RusRobotDriver::RobotState& state);

        /** 单帧字节数 */
        size_t FrameByteSize(uint32_t num_joints) const {
            return sizeof(double)                    /**< timestamp */
                + num_joints * sizeof(double) * 4   /**< joint_pos + joint_vel + joint_acc + effort */
                + 6 * sizeof(double);               /**< flange_pos (XYZABC) */
        }

        static constexpr uint32_t kMagic    = 0x52534452;  /**< "RSDR" */
        static constexpr uint32_t kVersion  = 1;

        std::ofstream file_;
        std::vector<RusRobotDriver::RobotState> frames_;  /**< 加载到内存，用于回放 */

        // ── 写线程 ──
        std::thread writer_thread_;
        std::atomic<bool> stop_writer_{false};
        std::deque<RusRobotDriver::RobotState> write_queue_;
        std::mutex queue_mtx_;
        std::condition_variable queue_cv_;

        uint32_t num_joints_ = 0;
        uint32_t written_frames_ = 0;
    };

}  // namespace RusRobotDriver
