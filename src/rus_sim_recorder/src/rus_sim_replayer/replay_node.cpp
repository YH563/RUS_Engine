#include "rus_sim_replayer/replay_node.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <limits>
#include <string>
#include <utility>

#include <rus_sim_utils/command_defs.hpp>

namespace RusRecorder {

    namespace {
        /// 倍速钳位（0 / 负数 / NaN → 保持原值；越界 → 收到 [kMinSpeed, kMaxSpeed]）
        double ClampSpeed(double s, double fallback)
        {
            if (!(s > 0.0)) return fallback;
            return std::min(std::max(s, ReplayNode::kMinSpeed), ReplayNode::kMaxSpeed);
        }

        /// 路径 → 文件名（日志 / replay_list 显示用）
        std::string BaseName(const std::string& path)
        {
            const auto pos = path.find_last_of('/');
            return pos == std::string::npos ? path : path.substr(pos + 1);
        }

        /// 参数是否为"非负整数"（序号类参数：1.5 / -1 都算非法）
        bool IsIndexArg(double v) { return v >= 0.0 && std::floor(v) == v; }

        /// ns → 秒（日志 / reply.result 用）
        double ToSec(int64_t ns) { return static_cast<double>(ns) / 1e9; }
    }  // namespace

    // ================================================================
    //  构造：读参数 → 起服务 / 回放线程 → 可选自动载入
    // ================================================================

    ReplayNode::ReplayNode(const rclcpp::NodeOptions& options)
        : Node("replayer_node", options)
    {
        // ── 参数 ──
        record_dir_          = declare_parameter<std::string>("record_dir", "records");
        file_path_           = declare_parameter<std::string>("file_path", "");
        file_prefix_         = declare_parameter<std::string>("file_prefix", "");
        file_index_          = declare_parameter<int>("file_index", 0);
        autoload_            = declare_parameter<bool>("autoload", true);
        autoplay_            = declare_parameter<bool>("autoplay", false);
        speed_               = declare_parameter<double>("speed", 1.0);
        loop_                = declare_parameter<bool>("loop", false);
        check_crc_           = declare_parameter<bool>("check_crc", true);
        use_recv_time_       = declare_parameter<bool>("use_recv_time", false);
        qos_depth_           = declare_parameter<int>("qos_depth", 10);
        qos_transient_local_ = declare_parameter<bool>("qos_transient_local", true);
        topic_prefix_        = declare_parameter<std::string>("topic_prefix", "");
        log_period_sec_      = declare_parameter<double>("log_period_sec", 5.0);

        speed_ = ClampSpeed(speed_, 1.0);
        qos_depth_ = std::min(std::max(qos_depth_, 1), 1000);
        if (record_dir_.empty()) record_dir_ = "records";
        while (record_dir_.size() > 1 && record_dir_.back() == '/') record_dir_.pop_back();
        // topic_prefix 归一：非空则 "/" 开头、不带尾 "/"（拼话题 = prefix + 原话题）
        if (!topic_prefix_.empty()) {
            if (topic_prefix_.front() != '/') topic_prefix_.insert(topic_prefix_.begin(), '/');
            while (topic_prefix_.size() > 1 && topic_prefix_.back() == '/') topic_prefix_.pop_back();
        }
        if (file_index_ < 0) file_index_ = 0;

        // ── 指令服务（bridge 注册表里的 Module::REPLAYER 目标）──
        cmd_server_ = create_service<CommandService>(
            "/replayer/command",
            std::bind(&ReplayNode::handle_command, this,
                      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

        // ── 事件（replay_done / error → bridge 转前端 event）──
        event_pub_ = create_publisher<rus_sim_interfaces::msg::ModuleEvent>("/module_events", 10);

        // ── 回放线程：唯一的发布者（指令线程只改状态 / 发单步）──
        playback_thread_ = std::thread(&ReplayNode::playback_loop, this);

        if (topic_prefix_.empty()) {
            RCLCPP_WARN(get_logger(),
                        "topic_prefix 为空：回放将原样发回录制时的话题（如 /driver/state）。"
                        "真驱动 / 真机在线时会撞话题，请先停驱动或用 topic_prefix:=/replay 隔离");
        }

        // ── 自动载入 / 自动播放 ──
        {
            std::lock_guard<std::mutex> lk(mtx_);
            if (autoload_) {
                std::string err;
                if (load_file(static_cast<size_t>(file_index_), &err)) {
                    if (autoplay_) {
                        state_ = State::kPlaying;
                        play_ack_id_ = 0;  // 自动播放无对应指令
                        anchor_locked(cursor_);
                    }
                } else {
                    RCLCPP_WARN(get_logger(), "启动载入失败：%s（可用 replay_load 显式载入）",
                                err.c_str());
                }
            }
        }
        cv_.notify_all();

        // ── 统计日志 ──
        log_timer_ = create_wall_timer(
            std::chrono::duration<double>(std::max(0.5, log_period_sec_)), [this]() {
                const uint64_t total = published_.load();
                const double delta = static_cast<double>(total - logged_published_);
                logged_published_ = total;
                std::lock_guard<std::mutex> lk(mtx_);
                if (state_ != State::kPlaying) return;
                RCLCPP_INFO(get_logger(),
                            "回放中：已发 %llu 条（%.1f 条/s）｜进度 %.1f / %.1f s｜倍速 %.2fx"
                            "｜失败 %llu",
                            static_cast<unsigned long long>(total),
                            delta / std::max(0.5, log_period_sec_),
                            ToSec(progress_ns_locked()), ToSec(duration_ns_locked()), speed_,
                            static_cast<unsigned long long>(failed_.load()));
            });

        RCLCPP_INFO(get_logger(), "ReplayNode 已启动：录音目录 %s，topic_prefix \"%s\"",
                    std::filesystem::absolute(record_dir_).string().c_str(),
                    topic_prefix_.c_str());
    }

    // ================================================================
    //  析构：停回放线程 → 释放文件（录音文件只读，不会被改动）
    // ================================================================

    ReplayNode::~ReplayNode()
    {
        {
            std::lock_guard<std::mutex> lk(mtx_);
            quit_ = true;
        }
        cv_.notify_all();
        if (playback_thread_.joinable()) playback_thread_.join();
        std::lock_guard<std::mutex> lk(mtx_);
        unload();
    }

    // ================================================================
    //  文件清单（每次载入 / replay_list 前刷新：录制期间目录会持续长新文件）
    // ================================================================

    void ReplayNode::refresh_file_list()
    {
        file_list_.clear();

        std::error_code ec;
        // file_path 指定：并入清单第 0 项（replay_load 0 即选中它）
        std::string pinned;
        if (!file_path_.empty()) {
            pinned = std::filesystem::absolute(file_path_, ec).string();
        }

        std::vector<std::string> names;
        if (std::filesystem::is_directory(record_dir_, ec)) {
            for (const auto& ent : std::filesystem::directory_iterator(record_dir_, ec)) {
                if (!ent.is_regular_file()) continue;
                if (ent.path().extension() != ".rusrec") continue;
                const std::string name = ent.path().filename().string();
                if (!file_prefix_.empty() && name.rfind(file_prefix_, 0) != 0) continue;
                names.push_back(name);
            }
        }
        std::sort(names.begin(), names.end());   // 文件名升序（含时间戳 → 即时间顺序）

        if (!pinned.empty()) file_list_.push_back(pinned);
        for (const auto& n : names) {
            const std::string path = std::filesystem::absolute(
                std::filesystem::path(record_dir_) / n, ec).string();
            if (path == pinned) continue;        // 已在第 0 项
            file_list_.push_back(path);
        }
    }

    // ================================================================
    //  卸载：释放发布器 / 时间轴 / 文件句柄，回到 idle
    // ================================================================

    void ReplayNode::unload()
    {
        pubs_.clear();   // 没有文件就不该继续有发布器（避免"半挂"状态下仍可发布）
        events_.clear();
        reader_.Close();
        loaded_ = false;
        loaded_path_.clear();
        cursor_ = 0;
        state_ = State::kIdle;
        t_origin_ns_ = 0;
        wall_origin_ = std::chrono::steady_clock::now();
        ++anchor_gen_;   // 唤醒回放线程重算（此时已无事件可发）
    }

    // ================================================================
    //  载入：建时间轴 + 建泛型发布器（失败保持"无文件"状态）
    // ================================================================

    bool ReplayNode::load_file(size_t file_index, std::string* err)
    {
        refresh_file_list();
        if (file_list_.empty()) {
            if (err) *err = "录音目录内没有 .rusrec 文件：" + record_dir_;
            return false;
        }
        if (file_index >= file_list_.size()) {
            if (err) *err = "文件序号越界：" + std::to_string(file_index) + " / " +
                            std::to_string(file_list_.size()) + "（用 replay_list 查看清单）";
            return false;
        }

        unload();   // 换文件前彻底释放上一个文件（发布器 / 时间轴 / 句柄）

        const std::string path = file_list_[file_index];
        std::string e;
        if (!reader_.Open(path, &e)) {
            if (err) *err = "打开录音失败：" + e;
            return false;
        }

        // ── 时间轴：只读记录元数据（payload 不读进内存，见 RecReader::BuildTimeline）──
        std::vector<Storage::RecReader::RecordMeta> metas;
        if (!reader_.BuildTimeline(metas, &e)) {
            if (err) *err = "扫描记录流失败：" + e;
            reader_.Close();
            return false;
        }
        if (metas.empty()) {
            if (err) *err = "文件里没有可回放的记录（0 条）";
            reader_.Close();
            return false;
        }

        // ── 通道 → 泛型发布器：话题名与消息类型名都来自录音的通道表 ──
        //    （transient_local 默认开：bridge 对 /sensor/pointcloud 的订阅要求
        //      publisher 端 durability ≥ transient_local，否则 DDS 不建立通路）
        rclcpp::QoS qos(qos_depth_);
        if (qos_transient_local_) qos.transient_local();
        for (const auto& ch : reader_.channels()) {
            const std::string topic = topic_prefix_ + ch.topic;
            try {
                ChannelPub cp;
                cp.pub = create_generic_publisher(topic, ch.type_name, qos);
                cp.topic = topic;
                cp.type_name = ch.type_name;
                pubs_[ch.channel_id] = std::move(cp);
            } catch (const std::exception& ex) {
                if (err) *err = "创建发布器失败（话题 " + topic + " / 类型 " + ch.type_name +
                                "）：" + ex.what();
                pubs_.clear();
                reader_.Close();
                return false;
            }
        }

        // ── 时间轴条目：选基准时间，平移到 0 起点，并强制单调（防倍速倒流）──
        events_.reserve(metas.size());
        for (const auto& m : metas) {
            Event ev;
            ev.offset = m.offset;
            ev.channel_id = m.channel_id;
            // 消息未带时间戳（stamp=0）时回退录制时刻，保证时间轴可用
            ev.t_ns = use_recv_time_ ? m.recv_ns : (m.stamp_ns != 0 ? m.stamp_ns : m.recv_ns);
            events_.push_back(ev);
        }
        const int64_t t0 = events_.front().t_ns;
        int64_t last_t = std::numeric_limits<int64_t>::min();
        for (auto& ev : events_) {
            ev.t_ns -= t0;
            if (ev.t_ns < last_t) ev.t_ns = last_t;   // 时间戳回退 → 钳到前一条
            last_t = ev.t_ns;
        }

        if (reader_.has_index() && reader_.index().size() != metas.size()) {
            RCLCPP_WARN(get_logger(),
                        "尾索引条数（%zu）与实际记录数（%zu）不一致：以顺序扫描的时间轴为准",
                        reader_.index().size(), metas.size());
        }

        loaded_ = true;
        loaded_path_ = path;
        loaded_index_ = file_index;
        cursor_ = 0;
        state_ = State::kPaused;   // 载入后停在起点（进度 0），等 replay_start
        anchor_locked(0);

        RCLCPP_INFO(get_logger(),
                    "已载入：%s（%zu 条记录 / %zu 通道 / 时长 %.2f s%s）",
                    path.c_str(), events_.size(), reader_.channels().size(),
                    ToSec(duration_ns_locked()),
                    reader_.has_index() ? "" : "；无尾索引：按顺序扫描的时间轴回放");
        if (!reader_.error().empty()) {
            RCLCPP_WARN(get_logger(), "扫描备注：%s", reader_.error().c_str());
        }
        return true;
    }

    // ================================================================
    //  事件（replayer → /module_events → bridge → 前端 event）
    // ================================================================

    void ReplayNode::publish_module_event(std::string_view event, bool success,
                                          std::string_view message,
                                          const std::vector<double>& result, uint32_t client_id)
    {
        rus_sim_interfaces::msg::ModuleEvent evt;
        evt.event = std::string(event);
        evt.success = success;
        evt.message = std::string(message);
        evt.result = result;
        evt.client_id = client_id;   // 关联触发指令的前端 command id（→ event.ack_id）
        evt.module = "replayer";
        evt.stamp = now();
        event_pub_->publish(evt);
    }

    // ================================================================
    //  时间轴辅助
    // ================================================================

    void ReplayNode::anchor_locked(size_t cursor)
    {
        wall_origin_ = std::chrono::steady_clock::now();
        t_origin_ns_ = events_.empty()
            ? 0
            : events_[std::min(cursor, events_.size() - 1)].t_ns;
        ++anchor_gen_;      // 代次变化 → 回放线程的 wait_until 立即醒来重算
        cv_.notify_all();
    }

    int64_t ReplayNode::progress_ns_locked() const
    {
        if (events_.empty()) return 0;
        if (cursor_ >= events_.size()) return events_.back().t_ns;  // 播完 = 全长
        return events_[cursor_].t_ns;
    }

    int64_t ReplayNode::duration_ns_locked() const
    {
        return events_.empty() ? 0 : events_.back().t_ns;   // 时间轴已平移到 0 起点
    }

    /// 定位：第一条 t_ns ≥ 给定值的记录（时间轴单调 → 二分）
    size_t ReplayNode::find_cursor_locked(int64_t t_ns) const
    {
        const auto it = std::lower_bound(events_.begin(), events_.end(), t_ns,
            [](const Event& e, int64_t v) { return e.t_ns < v; });
        return static_cast<size_t>(std::distance(events_.begin(), it));
    }

    /// 状态快照：[state, 进度 s, 时长 s, 倍速, 游标, 记录总数, 是否已载入, 文件序号, 文件总数]
    /// （字段顺序是协议的一部分，见 WsProtocol.md §4.7）
    std::vector<double> ReplayNode::status_locked() const
    {
        return {static_cast<double>(static_cast<int>(state_)),
                ToSec(progress_ns_locked()),
                ToSec(duration_ns_locked()),
                speed_,
                static_cast<double>(cursor_),
                static_cast<double>(events_.size()),
                loaded_ ? 1.0 : 0.0,
                static_cast<double>(loaded_index_),
                static_cast<double>(file_list_.size())};
    }

    // ================================================================
    //  发布：读回一条记录 → 泛型发布器原样重发 CDR 字节
    // ================================================================

    bool ReplayNode::publish_locked(size_t i, std::string* err)
    {
        const Event& ev = events_[i];
        const auto it = pubs_.find(ev.channel_id);
        if (it == pubs_.end()) {
            if (err) *err = "通道 " + std::to_string(ev.channel_id) + " 没有发布器";
            return false;
        }

        Storage::RecReader::RecordInfo info;
        if (!reader_.ReadRecordAt(ev.offset, info, payload_scratch_, check_crc_)) {
            if (err) *err = reader_.error();
            return false;
        }

        // payload 是录制时 rclcpp::Serialization 产出的完整 CDR（含 4 B 封装头）：
        // 原样重发，不做反序列化 → 前端看到的字节与录制时一致
        const size_t n = payload_scratch_.size();
        if (serialized_.capacity() < n) serialized_.reserve(n);
        auto& raw = serialized_.get_rcl_serialized_message();
        if (n > 0) std::memcpy(raw.buffer, payload_scratch_.data(), n);
        raw.buffer_length = n;
        it->second.pub->publish(serialized_);
        ++published_;
        return true;
    }



    // ================================================================
    //  回放线程：睡到时间轴上的点 → 读回一条 → 发布
    //  ────────────────────────────────────────────────────────────────
    //  它是唯一的节拍器；暂停 / 停止 / seek / 改倍速 / 退出都通过
    //  "状态 + 锚点代次" 立刻中断 wait_until（不会等到下一条记录才响应）。
    // ================================================================

    void ReplayNode::playback_loop()
    {
        std::unique_lock<std::mutex> lk(mtx_);
        while (!quit_) {
            cv_.wait(lk, [this] {
                return quit_ || (state_ == State::kPlaying && loaded_ && !events_.empty());
            });
            if (quit_) break;

            // 播到末尾：loop 则回起点，否则置 finished 并发 replay_done
            if (cursor_ >= events_.size()) {
                if (loop_) {
                    cursor_ = 0;
                    anchor_locked(0);
                    continue;
                }
                state_ = State::kFinished;
                const uint32_t ack = play_ack_id_;
                lk.unlock();
                publish_module_event(RusUtils::EventName::kReplayDone, true, "replay finished",
                                     {static_cast<double>(published_.load())}, ack);
                lk.lock();
                continue;
            }

            const uint64_t gen = anchor_gen_;
            const Event ev = events_[cursor_];
            // 目标墙钟 = 锚点墙钟 + (记录时刻 - 锚点时刻) / 倍速
            const auto target = wall_origin_ + std::chrono::nanoseconds(
                static_cast<int64_t>(static_cast<double>(ev.t_ns - t_origin_ns_) / speed_));
            if (cv_.wait_until(lk, target, [this, gen] {
                    return quit_ || state_ != State::kPlaying || anchor_gen_ != gen; })) {
                continue;   // 被打断（暂停 / 停止 / seek / 改倍速）→ 重新判定
            }

            const size_t idx = cursor_;
            std::string err;
            if (!publish_locked(idx, &err)) {
                state_ = State::kIdle;   // 读盘 / CRC 失败：停下来，别继续发坏数据
                ++failed_;
                const uint32_t ack = play_ack_id_;
                lk.unlock();
                publish_module_event(RusUtils::EventName::kError, false, "replay aborted: " + err,
                                     {}, ack);
                lk.lock();
                continue;
            }
            if (state_ == State::kPlaying && cursor_ == idx) ++cursor_;
        }
    }

    // ================================================================
    //  指令处理（/replayer/command；bridge 路由目标 Module::REPLAYER）
    //  ────────────────────────────────────────────────────────────────
    //  参数合法性由协议层（Cmd::ParseCommand）先过一遍；这里再对语义做校验
    //  （序号必须是整数、倍速必须 > 0 等）。所有读盘都在 mtx_ 内（RecReader 非线程安全）。
    // ================================================================

    void ReplayNode::handle_command(const std::shared_ptr<rmw_request_id_t> req_header,
                                    const std::shared_ptr<CommandService::Request> req,
                                    std::shared_ptr<CommandService::Response> res)
    {
        (void)req_header;
        using namespace RusUtils::CmdName;

        // ── 查询类 ──
        if (req->command == kReplayStatus) {
            std::lock_guard<std::mutex> lk(mtx_);
            res->success = true;
            res->message = loaded_ ? BaseName(loaded_path_) : "no file loaded";
            res->result = status_locked();
            if (loaded_) res->strings = {BaseName(loaded_path_)};   // 文本结果：当前文件名
            return;
        }

        if (req->command == kReplayList) {
            std::lock_guard<std::mutex> lk(mtx_);
            refresh_file_list();
            // strings = 机器可读清单（前端列表用）；message = 人类可读摘要（直接调服务时看）
            res->strings.clear();
            for (const auto& path : file_list_) res->strings.push_back(BaseName(path));
            res->success = !file_list_.empty();
            res->message = file_list_.empty()
                ? ("录音目录内没有 .rusrec 文件：" + record_dir_)
                : (std::to_string(file_list_.size()) + " 个录音文件（清单见 strings）");
            res->result = {static_cast<double>(file_list_.size()),
                           static_cast<double>(loaded_index_)};
            return;
        }

        // ── 载入 ──
        if (req->command == kReplayLoad) {
            std::lock_guard<std::mutex> lk(mtx_);
            size_t idx = loaded_index_;
            if (!req->args.empty()) {
                if (!IsIndexArg(req->args[0])) {
                    res->success = false;
                    res->message = "replay_load 的序号必须是非负整数";
                    return;
                }
                idx = static_cast<size_t>(req->args[0]);
            }
            std::string err;
            if (!load_file(idx, &err)) {
                res->success = false;
                res->message = err;
                RCLCPP_WARN(get_logger(), "replay_load 失败：%s", err.c_str());
                return;
            }
            res->success = true;
            res->message = "loaded: " + BaseName(loaded_path_);
            res->result = {static_cast<double>(file_list_.size()),
                           static_cast<double>(loaded_index_)};
            res->strings = {BaseName(loaded_path_)};
            return;
        }

        // ── 播放控制 ──
        if (req->command == kReplayStart) {
            std::lock_guard<std::mutex> lk(mtx_);
            if (!loaded_) {   // 未显式 load：按参数里的初始序号自动载入（前端一步起播）
                std::string err;
                if (!load_file(static_cast<size_t>(file_index_), &err)) {
                    res->success = false;
                    res->message = "尚未载入录音文件：" + err;
                    return;
                }
            }
            if (!req->args.empty()) {
                if (!(req->args[0] > 0.0)) {
                    res->success = false;
                    res->message = "倍速必须 > 0";
                    return;
                }
                speed_ = ClampSpeed(req->args[0], speed_);
            }
            if (cursor_ >= events_.size()) cursor_ = 0;   // 播完后重新从头
            state_ = State::kPlaying;
            play_ack_id_ = req->client_id;   // → replay_done 的 ack_id
            anchor_locked(cursor_);
            res->success = true;
            res->message = "playing";
            res->result = status_locked();
            RCLCPP_INFO(get_logger(), "开始回放：%s（倍速 %.2fx，%zu 条）",
                        BaseName(loaded_path_).c_str(), speed_, events_.size());
            return;
        }

        if (req->command == kReplayPause) {
            std::lock_guard<std::mutex> lk(mtx_);
            if (state_ != State::kPlaying) {
                res->success = false;
                res->message = "当前不在播放中";
                return;
            }
            state_ = State::kPaused;
            cv_.notify_all();   // 立即中断回放线程的等待
            res->success = true;
            res->message = "paused";
            res->result = status_locked();
            return;
        }

        if (req->command == kReplayResume) {
            std::lock_guard<std::mutex> lk(mtx_);
            if (state_ != State::kPaused) {
                res->success = false;
                res->message = "当前不在暂停中";
                return;
            }
            state_ = State::kPlaying;
            play_ack_id_ = req->client_id;
            anchor_locked(cursor_);   // 从暂停处继续（不跳时间）
            res->success = true;
            res->message = "resumed";
            res->result = status_locked();
            return;
        }



        if (req->command == kReplayStop) {
            std::lock_guard<std::mutex> lk(mtx_);
            state_ = State::kIdle;
            cursor_ = 0;
            anchor_locked(0);   // 复位到起点（文件保留，可直接再 start）
            res->success = true;
            res->message = "stopped";
            res->result = status_locked();
            return;
        }

        if (req->command == kReplaySeek) {
            std::lock_guard<std::mutex> lk(mtx_);
            if (!loaded_) {
                res->success = false;
                res->message = "尚未载入录音文件";
                return;
            }
            if (req->args.empty()) {
                res->success = false;
                res->message = "replay_seek 需要 1 个参数 [t_s]";
                return;
            }
            const double t = req->args[0];
            if (t < 0.0) {
                res->success = false;
                res->message = "目标时间必须 ≥ 0";
                return;
            }
            const int64_t dur = duration_ns_locked();
            const int64_t tgt = std::min<int64_t>(static_cast<int64_t>(t * 1e9), dur);
            cursor_ = find_cursor_locked(tgt);
            if (state_ == State::kFinished) state_ = State::kPaused;
            if (state_ == State::kPlaying) {
                anchor_locked(cursor_);   // 播放中跳转：以新位置重新起拍
            } else {
                cv_.notify_all();
            }
            res->success = true;
            res->message = "seeked";
            res->result = status_locked();
            return;
        }

        if (req->command == kReplaySetSpeed) {
            std::lock_guard<std::mutex> lk(mtx_);
            if (req->args.empty() || !(req->args[0] > 0.0)) {
                res->success = false;
                res->message = "replay_set_speed 需要 1 个 > 0 的参数 [speed]";
                return;
            }
            speed_ = ClampSpeed(req->args[0], speed_);
            if (state_ == State::kPlaying) anchor_locked(cursor_);   // 保持位置，新速度生效
            res->success = true;
            res->message = "speed set";
            res->result = {speed_};
            return;
        }

        if (req->command == kReplayStep) {
            std::lock_guard<std::mutex> lk(mtx_);
            if (!loaded_) {
                res->success = false;
                res->message = "尚未载入录音文件";
                return;
            }
            if (state_ == State::kPlaying) {
                res->success = false;
                res->message = "replay_step 仅在暂停 / 空闲态可用（先 replay_pause）";
                return;
            }
            size_t n = 1;
            if (!req->args.empty()) {
                if (!IsIndexArg(req->args[0]) || req->args[0] < 1.0) {
                    res->success = false;
                    res->message = "步数必须是 ≥1 的整数";
                    return;
                }
                n = static_cast<size_t>(req->args[0]);
            }
            size_t sent = 0;
            std::string err;
            while (sent < n && cursor_ < events_.size()) {
                if (!publish_locked(cursor_, &err)) {
                    state_ = State::kIdle;
                    ++failed_;
                    res->success = false;
                    res->message = "单步失败：" + err;
                    res->result = status_locked();
                    return;
                }
                ++cursor_;
                ++sent;
            }
            state_ = (cursor_ >= events_.size()) ? State::kFinished : State::kPaused;
            res->success = true;
            res->message = "stepped " + std::to_string(sent);
            res->result = status_locked();
            return;
        }

        // ── 未注册指令：bridge 已校验一次，这里是服务被直接调用时的兜底 ──
        res->success = false;
        res->message = "unknown command: " + req->command;
        RCLCPP_WARN(get_logger(), "未知指令：%s", req->command.c_str());
    }

}  // namespace RusRecorder
