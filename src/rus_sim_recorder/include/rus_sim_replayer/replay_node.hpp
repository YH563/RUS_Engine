#pragma once

// ════════════════════════════════════════════════════════════════════
//  回放节点（rus_sim_recorder：按 .rusrec 时间轴把录音重发回话题）
//  ────────────────────────────────────────────────────────────────────
//  与记录侧镜像：写盘 → 读盘 + 发布。
//
//  数据流：
//    载入（replay_load）：目录清单 → Open() → BuildTimeline()（只读记录元数据，
//        不把 payload 读进内存）→ 按文件里的通道表建立泛型发布器（topic / type
//        都来自录音，故任何录过的消息类型都能回放）
//    回放线程：睡到时间轴上的点 → ReadRecordAt()（读回 + 可选 CRC 校验）→
//        泛型发布器原样重发 CDR 字节（**不重打时间戳、不改 payload**）
//    指令线程（/replayer/command）：load / list / start / pause / resume / stop /
//        seek / set_speed / step / status（路由与参数见 WsProtocol.md §4.7）
//
//  两条口径（详见 docs/Protocol/RecFormat.md §7.4）：
//    1. 时间轴：time_source=stamp（默认，消息时间戳）/ recv（录制入队时刻）；
//       倍速与 seek 都作用在这条轴上；时间戳回退（map_clear / 通道交错）会被钳到
//       前一条，保证倍速播放不倒流。
//    2. 话题：默认按录制时的话题原样发布（前端可视化路径）；与真机 / 真驱动共存时
//       用 topic_prefix 隔离（如 /replay），避免撞话题。
//
//  崩溃录音（无尾索引）同样可回放：时间轴由顺序扫描建立，payload 按偏移逐条读回。
//  RecReader 非线程安全 → 所有读盘都在 mtx_ 内（回放线程 / 指令线程互斥）。
// ════════════════════════════════════════════════════════════════════

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/generic_publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialized_message.hpp>

#include <rus_sim_interfaces/msg/module_event.hpp>
#include <rus_sim_interfaces/srv/command_service.hpp>

#include "storage/rec_reader.hpp"

namespace RusRecorder {

    /// 回放节点（把 `.rusrec` 按时间轴重发回录制时的话题）
    class ReplayNode : public rclcpp::Node {
    public:
        explicit ReplayNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
        ~ReplayNode() override;  // 停回放线程 → 关文件

        /// 倍速范围（协议层同口径，见 WsProtocol.md §4.7）
        static constexpr double kMinSpeed = 0.05;
        static constexpr double kMaxSpeed = 20.0;

    private:
        using CommandService = rus_sim_interfaces::srv::CommandService;

        /// 播放状态（对外编码：0=idle 1=playing 2=paused 3=finished）
        enum class State : int { kIdle = 0, kPlaying = 1, kPaused = 2, kFinished = 3 };

        /// 时间轴条目（payload 不驻留内存：发布前一刻按 offset 读回）
        struct Event {
            int64_t offset = 0;      // 记录头绝对偏移
            int64_t t_ns = 0;        // 播放时刻（相对文件起点；time_source 决定取值来源）
            uint16_t channel_id = 0;
        };

        /// 通道 → 泛型发布器（载入时按录音里的通道表一次建好；发布侧只 publish）
        struct ChannelPub {
            rclcpp::GenericPublisher::SharedPtr pub;
            std::string topic;      // 实际发布话题（含 topic_prefix）
            std::string type_name;
        };

        // ── 指令（/replayer/command）──
        void handle_command(const std::shared_ptr<rmw_request_id_t> req_header,
                            const std::shared_ptr<CommandService::Request> req,
                            std::shared_ptr<CommandService::Response> res);

        // ── 文件清单 / 载入（未注明"无锁"者均要求调用者持 mtx_）──
        void refresh_file_list();
        /// 载入第 file_index 个文件（建时间轴 + 建发布器）；失败保持"无文件"状态
        bool load_file(size_t file_index, std::string* err);
        void unload();

        // ── 回放 ──
        void playback_loop();
        /// 以 cursor 处记录为时间锚点（暂停 / 恢复 / seek / 改倍速后调用）
        void anchor_locked(size_t cursor);
        /// 读回第 i 条并发布（持锁：RecReader 非线程安全；回放线程与 step 共用）
        bool publish_locked(size_t i, std::string* err);
        /// 发布 /module_events（bridge 转成前端 event；client_id → event.ack_id）
        void publish_module_event(std::string_view event, bool success, std::string_view message,
                                  const std::vector<double>& result, uint32_t client_id);

        // ── 状态快照（持锁）──
        std::vector<double> status_locked() const;
        int64_t progress_ns_locked() const;
        int64_t duration_ns_locked() const;
        size_t find_cursor_locked(int64_t t_ns) const;

        // ── 参数 ──
        std::string record_dir_;            // 录音目录（与 recorder 的 output_dir 对齐）
        std::string file_path_;             // 非空：直接指定文件（并入清单第 0 项）
        std::string file_prefix_;           // 目录过滤前缀（"" = 全部 .rusrec）
        int file_index_ = 0;                // file_path 为空时的初始文件序号
        bool autoload_ = true;              // 启动即载入
        bool autoplay_ = false;             // 载入后立即播放
        double speed_ = 1.0;                // 初始倍速
        bool loop_ = false;                 // 播完循环
        bool check_crc_ = true;             // 发布前逐条校验 payload CRC
        bool use_recv_time_ = false;        // 时间轴基准：false=stamp，true=recv
        int qos_depth_ = 10;                // 发布队列深度
        bool qos_transient_local_ = true;   // 与 /sensor/pointcloud 的 transient_local 订阅兼容
        std::string topic_prefix_;          // 发布话题前缀（"" = 原样）
        double log_period_sec_ = 5.0;       // 回放统计日志周期

        // ── 共享状态（mtx_ 保护）──
        std::mutex mtx_;
        std::condition_variable cv_;
        bool quit_ = false;
        State state_ = State::kIdle;
        bool loaded_ = false;
        std::string loaded_path_;           // 当前文件完整路径
        size_t loaded_index_ = 0;           // 当前文件在 file_list_ 中的下标
        Storage::RecReader reader_;
        std::vector<Event> events_;
        std::map<uint16_t, ChannelPub> pubs_;
        std::vector<std::string> file_list_;  // 完整路径（显示取 basename）
        size_t cursor_ = 0;                 // 下一条待发布的记录
        uint32_t play_ack_id_ = 0;          // 触发播放的指令 id → replay_done 的 ack_id
        uint64_t anchor_gen_ = 0;           // 锚点代次：变化即唤醒回放线程重算
        std::chrono::steady_clock::time_point wall_origin_{};
        int64_t t_origin_ns_ = 0;

        // ── 复用缓冲（持锁访问）──
        std::vector<uint8_t> payload_scratch_;
        rclcpp::SerializedMessage serialized_;

        // ── ROS 接口 ──
        rclcpp::Service<CommandService>::SharedPtr cmd_server_;
        rclcpp::Publisher<rus_sim_interfaces::msg::ModuleEvent>::SharedPtr event_pub_;
        rclcpp::TimerBase::SharedPtr log_timer_;
        std::thread playback_thread_;

        // ── 统计 ──
        std::atomic<uint64_t> published_{0};
        std::atomic<uint64_t> failed_{0};
        uint64_t logged_published_ = 0;     // 仅日志定时器访问
    };

}  // namespace RusRecorder
