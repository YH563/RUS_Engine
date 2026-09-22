#include "rus_sim_recorder/recorder_node.hpp"

#include <algorithm>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <system_error>
#include <utility>

namespace RusRecorder {

    namespace {
        /// 本机系统时钟（纳秒）：与消息时间戳（ROS 时间）不是同一时基，分开存
        int64_t NowUnixNs()
        {
            return std::chrono::duration_cast<std::chrono::nanoseconds>(
                       std::chrono::system_clock::now().time_since_epoch())
                .count();
        }

        /// builtin_interfaces/Time → 纳秒（消息自带时间戳；0 = 未设置）
        int64_t StampNs(const builtin_interfaces::msg::Time& t)
        {
            return static_cast<int64_t>(t.sec) * 1000000000LL + static_cast<int64_t>(t.nanosec);
        }

        /// 文件名时间戳：20260922_141530
        std::string LocalStamp()
        {
            const std::time_t tt = std::chrono::system_clock::to_time_t(
                std::chrono::system_clock::now());
            std::tm tm{};
            localtime_r(&tt, &tm);
            char buf[32] = {0};
            std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &tm);
            return buf;
        }

        /// payload 编码方式（当前只有 ROS 消息 CDR 一种）
        constexpr uint16_t kKindRosMsg = static_cast<uint16_t>(Components::PayloadKind::kRosMsg);
    }  // namespace

    RecorderNode::RecorderNode() : Node("recorder_node")
    {
        // ── 输出 / 落盘参数 ──
        enabled_            = declare_parameter<bool>("enabled", true);
        output_dir_         = declare_parameter<std::string>("output_dir", "records");
        file_prefix_        = declare_parameter<std::string>("file_prefix", "run");
        const int max_mb    = declare_parameter<int>("max_file_size_mb", 512);
        flush_interval_sec_ = declare_parameter<double>("flush_interval_sec", 1.0);
        queue_max_records_  = static_cast<size_t>(declare_parameter<int>("queue_max", 2048));
        const int queue_mb  = declare_parameter<int>("queue_max_mb", 256);
        qos_depth_          = declare_parameter<int>("qos_depth", 20);
        log_period_sec_     = declare_parameter<double>("log_period_sec", 5.0);

        // ── 通道参数 ──
        record_state_       = declare_parameter<bool>("record_state", true);
        state_topic_        = declare_parameter<std::string>("state_topic", "/driver/state");
        state_max_rate_hz_  = declare_parameter<double>("state_max_rate_hz", 0.0);
        record_sensor_      = declare_parameter<bool>("record_sensor", true);
        sensor_topic_       = declare_parameter<std::string>("sensor_topic", "/sensor/pointcloud");
        sensor_max_rate_hz_ = declare_parameter<double>("sensor_max_rate_hz", 0.0);

        max_file_size_bytes_ = max_mb > 0
            ? static_cast<uint64_t>(max_mb) * 1024u * 1024u : 0u;
        queue_max_bytes_ = queue_mb > 0
            ? static_cast<size_t>(queue_mb) * 1024u * 1024u : 0u;
        queue_max_records_ = std::max<size_t>(1, queue_max_records_);
        qos_depth_ = std::min(std::max(qos_depth_, 1), 1000);

        if (!enabled_) {
            RCLCPP_WARN(get_logger(), "录制未启用（enabled=false）：不订阅、不落盘");
            return;
        }
        if (!record_state_ && !record_sensor_) {
            RCLCPP_WARN(get_logger(), "两个通道都关闭（record_state / record_sensor 均 false），无数据可录");
            return;
        }

        // 输出目录（相对路径按工作目录解析 → 日志里打印绝对路径，避免找不到文件）
        if (output_dir_.empty()) output_dir_ = "records";
        std::error_code ec;
        std::filesystem::create_directories(output_dir_, ec);
        if (ec && !std::filesystem::is_directory(output_dir_)) {
            RCLCPP_ERROR(get_logger(), "输出目录不可用：%s（%s）→ 录制关闭",
                         output_dir_.c_str(), ec.message().c_str());
            return;
        }
        output_dir_ = std::filesystem::absolute(output_dir_).string();
        run_stamp_ = LocalStamp();

        // ── 通道表 ──
        if (record_state_) {
            Components::ChannelDesc ch;
            ch.channel_id = kChannelState;
            ch.kind = kKindRosMsg;
            ch.topic = state_topic_;
            ch.type_name = "rus_sim_interfaces/msg/RobotState";
            ch.note = "机械臂状态（关节 / 法兰 / TCP）";
            channels_.push_back(ch);
        }
        if (record_sensor_) {
            Components::ChannelDesc ch;
            ch.channel_id = kChannelSensor;
            ch.kind = kKindRosMsg;
            ch.topic = sensor_topic_;
            ch.type_name = "rus_sim_interfaces/msg/SensorFrame";
            ch.note = "感知帧（压缩点云，scope 区分当前帧 / 地图快照）";
            channels_.push_back(ch);
        }

        // ── 订阅（reliable + volatile：与 driver / perception 的发布端兼容；
        //      不请求 transient_local —— 录的是实时流，不是订阅瞬间的历史样本）──
        const rclcpp::QoS qos(rclcpp::KeepLast(static_cast<size_t>(qos_depth_)));
        if (record_state_) {
            state_sub_ = create_subscription<RobotStateMsg>(
                state_topic_, qos,
                [this](const RobotStateMsg::SharedPtr msg) { on_state(msg); });
        }
        if (record_sensor_) {
            sensor_sub_ = create_subscription<SensorFrame>(
                sensor_topic_, qos,
                [this](const SensorFrame::SharedPtr msg) { on_sensor(msg); });
        }

        // ── 写线程（打开文件、出队、落盘都在里面）──
        writer_thread_ = std::thread(&RecorderNode::writer_loop, this);

        // ── 统计日志 ──
        log_timer_ = create_wall_timer(std::chrono::duration<double>(std::max(0.5, log_period_sec_)),
                                       [this] { log_stats(); });

        std::ostringstream ch_desc;
        for (size_t i = 0; i < channels_.size(); ++i) {
            if (i) ch_desc << ", ";
            ch_desc << channels_[i].channel_id << ":" << channels_[i].topic;
        }
        RCLCPP_INFO(get_logger(), "录制已启动：%s/%s_<时间戳>.rusrec，通道 %s",
                    output_dir_.c_str(), file_prefix_.c_str(), ch_desc.str().c_str());
    }

    RecorderNode::~RecorderNode()
    {
        if (log_timer_) log_timer_->cancel();
        {
            std::lock_guard<std::mutex> lk(mtx_);
            stop_ = true;
        }
        cv_.notify_all();
        if (writer_thread_.joinable()) writer_thread_.join();  // 线程内排空队列 + 写尾索引

        if (enabled_ && (record_state_ || record_sensor_)) {
            RCLCPP_INFO(get_logger(), "录制结束：累计 %llu 条 / %.2f MiB（丢弃 %llu，限流 %llu）",
                        static_cast<unsigned long long>(total_records_.load()),
                        static_cast<double>(total_payload_bytes_.load()) / (1024.0 * 1024.0),
                        static_cast<unsigned long long>(dropped_.load()),
                        static_cast<unsigned long long>(throttled_.load()));
        }
    }

    // ================================================================
    //  订阅回调（执行器线程）：只做序列化 + 入队
    // ================================================================

    void RecorderNode::on_state(const RobotStateMsg::SharedPtr msg)
    {
        rclcpp::Serialization<RobotStateMsg> ser;
        rclcpp::SerializedMessage sm;
        ser.serialize_message(msg.get(), &sm);
        const auto& raw = sm.get_rcl_serialized_message();
        enqueue(kChannelState, StampNs(msg->header.stamp), raw.buffer, raw.buffer_length,
                state_max_rate_hz_);
    }

    void RecorderNode::on_sensor(const SensorFrame::SharedPtr msg)
    {
        rclcpp::Serialization<SensorFrame> ser;
        rclcpp::SerializedMessage sm;
        ser.serialize_message(msg.get(), &sm);
        const auto& raw = sm.get_rcl_serialized_message();
        enqueue(kChannelSensor, StampNs(msg->stamp), raw.buffer, raw.buffer_length,
                sensor_max_rate_hz_);
    }

    void RecorderNode::enqueue(uint16_t channel, int64_t stamp_ns, const uint8_t* data, size_t size,
                               double max_rate_hz)
    {
        if (write_failed_.load()) {
            dropped_.fetch_add(1);
            return;
        }
        {
            std::lock_guard<std::mutex> lk(mtx_);

            // 限流（可选）：按墙钟最小间隔丢，避免把 125Hz 状态全量写盘
            if (max_rate_hz > 0.0) {
                const auto now = std::chrono::steady_clock::now();
                auto it = last_accept_.find(channel);
                if (it != last_accept_.end() &&
                    std::chrono::duration<double>(now - it->second).count() < 1.0 / max_rate_hz) {
                    throttled_.fetch_add(1);
                    return;
                }
                last_accept_[channel] = now;
            }

            // 有界队列：条数 / 字节任一超限都丢新（宁可丢数据，也不阻塞发布端）
            if (queue_.size() >= queue_max_records_ ||
                (queue_max_bytes_ > 0 && queue_bytes_ + size > queue_max_bytes_)) {
                dropped_.fetch_add(1);
                return;
            }

            QueuedRecord rec;
            rec.channel = channel;
            rec.kind = kKindRosMsg;
            rec.stamp_ns = stamp_ns;
            rec.recv_ns = NowUnixNs();
            if (size > 0) rec.payload.assign(data, data + size);
            queue_.push_back(std::move(rec));
            queue_bytes_ += size;
        }
        cv_.notify_one();
    }

    // ================================================================
    //  写线程：独占 RecWriter（打开 / 出队 / 落盘 / 滚动 / 封存）
    // ================================================================

    void RecorderNode::writer_loop()
    {
        if (!open_output_file(true)) {
            write_failed_.store(true);
            RCLCPP_ERROR(get_logger(), "录制文件创建失败，录制终止：%s", writer_.error().c_str());
            return;
        }

        const auto flush_interval = std::chrono::duration<double>(flush_interval_sec_);
        auto last_flush = std::chrono::steady_clock::now();
        while (true) {
            std::deque<QueuedRecord> batch;
            {
                std::unique_lock<std::mutex> lk(mtx_);
                // 有数据立刻取走；空闲时也按 20ms 醒来（保证 flush / 滚动不被延迟）
                cv_.wait_for(lk, std::chrono::milliseconds(20),
                             [this] { return stop_ || !queue_.empty(); });
                if (queue_.empty() && stop_) break;  // 退出前把队列排空
                batch.swap(queue_);
                queue_bytes_ = 0;
            }

            for (auto& rec : batch) {
                if (!writer_.WriteRecord(rec.channel, rec.stamp_ns, rec.recv_ns,
                                         rec.payload.data(), rec.payload.size())) {
                    RCLCPP_ERROR(get_logger(), "写记录失败（录制终止）：%s", writer_.error().c_str());
                    write_failed_.store(true);
                    break;
                }
                total_records_.fetch_add(1);
                total_payload_bytes_.fetch_add(rec.payload.size());
                current_file_bytes_.store(writer_.Offset());  // 供统计日志跨线程读
                // 单文件到顶：就地封存（写尾索引）后换新文件 —— 本批剩余记录自然写进新文件，
                // 不丢数据（不 break 出整批）
                if (max_file_size_bytes_ > 0 && writer_.Offset() >= max_file_size_bytes_) {
                    close_output_file();
                    if (!open_output_file(false)) {
                        write_failed_.store(true);
                        RCLCPP_ERROR(get_logger(), "滚动新文件失败（录制终止）：%s",
                                     writer_.error().c_str());
                        break;
                    }
                    last_flush = std::chrono::steady_clock::now();
                }
            }

            if (write_failed_.load()) break;
            const auto now = std::chrono::steady_clock::now();
            if (now - last_flush >= flush_interval) {
                writer_.Flush();
                last_flush = now;
            }
        }

        close_output_file();
    }

    bool RecorderNode::open_output_file(bool first)
    {
        std::string name = file_prefix_ + "_" + run_stamp_;
        if (!first) {
            std::ostringstream oss;
            oss << "_p" << std::setw(3) << std::setfill('0') << ++roll_index_;
            name += oss.str();
        }
        const std::filesystem::path path =
            std::filesystem::path(output_dir_) / (name + ".rusrec");

        Storage::RecWriter::Options opt;
        opt.path = path.string();
        bool ok = writer_.Open(opt);
        for (size_t i = 0; ok && i < channels_.size(); ++i) {
            ok = writer_.AddChannel(channels_[i]);
        }
        if (ok) ok = writer_.Start();
        if (!ok) {
            writer_.Close();  // 尽力收尾（内容不完整，但至少刷盘）
            file_active_ = false;
            return false;
        }

        file_active_ = true;
        const std::string limit = max_file_size_bytes_ == 0
            ? std::string("不限")
            : std::to_string(max_file_size_bytes_ / (1024 * 1024)) + " MiB";
        RCLCPP_INFO(get_logger(), "录制文件已打开：%s（%zu 通道，单文件上限 %s）",
                    path.c_str(), channels_.size(), limit.c_str());
        return true;
    }

    void RecorderNode::close_output_file()
    {
        if (!file_active_) return;
        const std::string path = writer_.path();
        const uint64_t records = writer_.stats().records;
        const uint64_t bytes = writer_.stats().payload_bytes;
        if (writer_.Close()) {
            RCLCPP_INFO(get_logger(), "录制文件已封存（含尾索引）：%s（%llu 条 / %.2f MiB）",
                        path.c_str(), static_cast<unsigned long long>(records),
                        static_cast<double>(bytes) / (1024.0 * 1024.0));
        } else {
            write_failed_.store(true);
            RCLCPP_ERROR(get_logger(), "文件封存失败：%s（%s），可用 rus_sim_recorder_inspect --scan 抢救",
                         path.c_str(), writer_.error().c_str());
        }
        file_active_ = false;
    }

    void RecorderNode::log_stats()
    {
        const uint64_t total = total_records_.load();
        const uint64_t bytes = total_payload_bytes_.load();
        const double d_rec = static_cast<double>(total - logged_records_);
        const double d_bytes = static_cast<double>(bytes - logged_bytes_);
        logged_records_ = total;
        logged_bytes_ = bytes;

        size_t q_len = 0;
        size_t q_bytes = 0;
        {
            std::lock_guard<std::mutex> lk(mtx_);
            q_len = queue_.size();
            q_bytes = queue_bytes_;
        }
        const double period = std::max(0.5, log_period_sec_);
        RCLCPP_INFO(get_logger(),
                    "录制中：%llu 条 / %.2f MiB（%.1f 条/s，%.0f KiB/s）"
                    "｜队列 %zu 条 / %.1f MiB｜丢弃 %llu，限流 %llu｜当前文件 %.2f MiB",
                    static_cast<unsigned long long>(total),
                    static_cast<double>(bytes) / (1024.0 * 1024.0),
                    d_rec / period, d_bytes / period / 1024.0,
                    q_len, static_cast<double>(q_bytes) / (1024.0 * 1024.0),
                    static_cast<unsigned long long>(dropped_.load()),
                    static_cast<unsigned long long>(throttled_.load()),
                    static_cast<double>(current_file_bytes_.load()) / (1024.0 * 1024.0));
    }

}  // namespace RusRecorder


