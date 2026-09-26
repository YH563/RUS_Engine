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
//  录制开关（运行期可控）：
//    默认 autostart=true → 进程一起来就录（等价于收到一次 recorder_start）；
//    autostart=false 时起来处于 stopped，由外部指令开始（见下）。
//    /recorder/command（bridge 路由目标 Module::RECORDER，协议见 WsProtocol.md §4.8）：
//      recorder_start   开始录制（新文件；已在录制 / 不可用 → 失败）
//      recorder_stop    停录：把已入队数据写完 → 封存（写尾索引），文件保留可再 start
//      recorder_status  查询状态（result 7 项 + strings=[当前 / 最后文件名]）
//    ⇒ 写线程常驻，"是否落盘" 由 recording_ 期望状态驱动：回调在停录期间不入队
//      （静默丢弃，不计入 dropped —— 那是"异常丢数据"的计数）。
//
//  落盘（格式见 docs/Protocol/RecFormat.md）：
//    records/run_<时间戳>.rusrec           单文件：[FileHeader][通道表][记录…][尾索引][Footer]
//    records/run_<时间戳>_p002.rusrec      超过 max_file_size_mb 后滚动；stop→start 也换新文件
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
#include <rus_sim_interfaces/srv/command_service.hpp>

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

        /// 对外录制状态编码（`recorder_status.result[0]`，协议见 WsProtocol.md §4.8）
        enum class State : int {
            kStopped = 0,    // 未录制（未启动 / 已 recorder_stop）
            kRecording = 1,  // 录制中
            kFailed = 2,     // 写失败熔断（需检查磁盘后重启节点）
        };

    private:
        using CommandService = rus_sim_interfaces::srv::CommandService;
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
        /// 打开输出文件（plain_name=true 不加滚动序号后缀；失败置 write_failed_）
        bool open_output_file(bool plain_name);
        /// 封存当前文件（写尾索引）；err 非空时回填失败原因。返回是否成功
        bool close_output_file(std::string* err = nullptr);
        void log_stats();

        // ── 指令（/recorder/command：运行期开 / 关落盘，见 WsProtocol.md §4.8）──
        void handle_command(const std::shared_ptr<rmw_request_id_t> req_header,
                            const std::shared_ptr<CommandService::Request> req,
                            std::shared_ptr<CommandService::Response> res);
        /// 请求写线程切换到目标录制状态并等它完成（成功返回 true，失败说明写入 err）
        bool request_recording(bool on, std::string* err);
        /// 状态快照（result 7 项；顺序是协议的一部分，见 WsProtocol.md §4.8）
        std::vector<double> status_snapshot() const;
        /// 写线程完成一次开 / 封后回填结果并唤醒等待中的 service 线程
        void finish_state_change(const std::string& err, const std::string& file_path);

        // ── 参数 ──
        bool enabled_ = true;              // false = 不订阅不落盘（进程空转）
        bool autostart_ = true;            // true = 启动即录；false = 起来处于 stopped，等 recorder_start
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
        uint32_t roll_index_ = 0; // 滚动序号（文件名 _pNNN 后缀；0 = 本次运行第一个文件）
        bool file_active_ = false;

        // ── 录制开关（外部控制：service 线程 ↔ 写线程）──
        std::atomic<bool> ready_{false};      // 可开始录制（enabled && 有通道 && 目录可用）
        std::atomic<bool> recording_{false};  // 期望状态：是否在录（写线程据此开 / 封文件）
        std::atomic<bool> state_req_{false};  // 有未处理的状态切换请求（唤醒写线程）
        std::mutex cmd_mtx_;                  // 服务回调串行化（多线程执行器下也不会并发开 / 封）
        // 状态切换握手：service 线程置 state_busy_ 后等写线程回填
        std::mutex state_mtx_;
        std::condition_variable state_cv_;
        bool state_busy_ = false;     // 写线程尚未完成本次切换
        std::string state_err_;       // 切换结果（空 = 成功）
        std::string state_file_;      // 当前 / 最后封存的文件完整路径

        // ── ROS 接口 ──
        rclcpp::Service<CommandService>::SharedPtr cmd_server_;

        // ── 统计 ──
        std::atomic<uint64_t> files_created_{0};  // 已打开的录制文件数（含当前）
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
