#pragma once

// ════════════════════════════════════════════════════════════════════
//  记录节点（rus_sim_recorder：运行时数据流落盘）
//  ────────────────────────────────────────────────────────────────────
//  双路录制（通道号固定，离线工具依赖）：
//    通道 0  /driver/state       RobotState   （125Hz）机械臂全状态
//    通道 1  /sensor/pointcloud  SensorFrame  （与感知处理同频）压缩感知帧
//
//  数据流：
//    订阅回调（执行器线程）  CDR 序列化 → 入队（有界：条数 + 字节双上限）
//    写线程（独立线程）      出队 → CRC32 → 写缓冲 → 定期 flush / 超限滚动新文件
//    ⇒ 回调绝不碰磁盘：录制不拖慢发布端；队列满则丢新并计数（宁可丢，不阻塞）
//
//  落盘（格式见 docs/Protocol/RecFormat.md）：
//    records/run_<时间戳>.rusrec           单文件：[FileHeader][通道表][记录…][尾索引][Footer]
//    records/run_<时间戳>_p002.rusrec      超过 max_file_size_mb 后滚动
//  正常关闭（含 Ctrl-C）写尾索引；进程被强杀的文件无尾索引，仍可顺序扫描读回。
// ════════════════════════════════════════════════════════════════════

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>

#include <rus_sim_interfaces/msg/robot_state.hpp>
#include <rus_sim_interfaces/msg/sensor_frame.hpp>

#include "components/rec_format.hpp"
#include "storage/rec_writer.hpp"

namespace RusRecorder {

    using RobotStateMsg = rus_sim_interfaces::msg::RobotState;
    using SensorFrame = rus_sim_interfaces::msg::SensorFrame;

    /// 记录节点（把 /driver/state 与 /sensor/pointcloud 落成 .rusrec）
    class RecorderNode : public rclcpp::Node {
    public:
        RecorderNode();
        ~RecorderNode() override;  // 停写线程 → 排空队列 → 写尾索引（Ctrl-C 也走这里）

    private:
        // 通道号（写入文件头通道表，离线工具据此解释 payload）
        static constexpr uint16_t kChannelState = 0;
        static constexpr uint16_t kChannelSensor = 1;

        /// 队列元素：payload 已序列化（回调里完成，写线程只管落盘）
        struct QueuedRecord {
            uint16_t channel = 0;
            uint16_t kind = 0;
            int64_t stamp_ns = 0;
            int64_t recv_ns = 0;
            std::vector<uint8_t> payload;
        };

        // ── 订阅回调（执行器线程）──
        void on_state(const RobotStateMsg::SharedPtr msg);
        void on_sensor(const SensorFrame::SharedPtr msg);
        /// 序列化后的字节入队（限流 / 队列上限判定都在这里，一把锁）
        void enqueue(uint16_t channel, int64_t stamp_ns, const uint8_t* data, size_t size,
                     double max_rate_hz);

        // ── 写线程（独占 RecWriter）──
        void writer_loop();
        bool open_output_file(bool first);
        void close_output_file();
        void log_stats();

        // ── 参数 ──
        bool enabled_ = true;              // false = 不订阅不落盘（进程空转）
        std::string output_dir_;           // 输出目录（相对路径按工作目录解析）
        std::string file_prefix_;          // 文件名前缀
        uint64_t max_file_size_bytes_ = 0; // 单文件上限（0 = 不限）
        double flush_interval_sec_ = 1.0;  // 缓冲落盘周期
        size_t queue_max_records_ = 0;     // 队列条数上限
        size_t queue_max_bytes_ = 0;       // 队列字节上限（防大帧把内存吃爆）
        int qos_depth_ = 20;               // 订阅队列深度
        double log_period_sec_ = 5.0;      // 统计日志周期
        bool record_state_ = true;
        std::string state_topic_;
        double state_max_rate_hz_ = 0.0;   // 0 = 不限（全量录）
        bool record_sensor_ = true;
        std::string sensor_topic_;
        double sensor_max_rate_hz_ = 0.0;

        // ── 通道表（写入文件头）──
        std::vector<Components::ChannelDesc> channels_;

        // ── 订阅 ──
        rclcpp::Subscription<RobotStateMsg>::SharedPtr state_sub_;
        rclcpp::Subscription<SensorFrame>::SharedPtr sensor_sub_;

        // ── 队列（回调线程 ↔ 写线程）──
        std::deque<QueuedRecord> queue_;
        std::mutex mtx_;
        std::condition_variable cv_;
        bool stop_ = false;
        size_t queue_bytes_ = 0;
        std::map<uint16_t, std::chrono::steady_clock::time_point> last_accept_;  // 限流用

        // ── 写线程状态（仅写线程访问）──
        std::thread writer_thread_;
        Storage::RecWriter writer_;
        std::string run_stamp_;   // 本次运行时间戳（文件名）
        uint32_t roll_index_ = 0; // 滚动序号
        bool file_active_ = false;

        // ── 统计 ──
        std::atomic<uint64_t> dropped_{0};    // 队列满 / 写失败丢弃的记录数
        std::atomic<uint64_t> throttled_{0};  // 限流丢弃的记录数
        std::atomic<bool> write_failed_{false};
        std::atomic<uint64_t> total_records_{0};
        std::atomic<uint64_t> total_payload_bytes_{0};
        std::atomic<uint64_t> current_file_bytes_{0};  // 写线程更新，日志线程只读
        uint64_t logged_records_ = 0;         // 仅日志定时器访问
        uint64_t logged_bytes_ = 0;
        rclcpp::TimerBase::SharedPtr log_timer_;
    };

}  // namespace RusRecorder
