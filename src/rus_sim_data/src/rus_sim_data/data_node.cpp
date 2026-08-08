#include "rus_sim_data/data_node.hpp"

#include <cmath>
#include <utility>

namespace RusSimData {

    namespace {

        /** 回放状态发布频率 [Hz] */
        constexpr double kPlaybackRate = 125.0;

        /** WsServer 广播端口（与驱动 8765 区分） */
        constexpr int kWsPort = 8766;

        /** 回放信息格式：[current_frame, total_frames, current_time, total_time, speed, progress, playing] */
        constexpr int kPlaybackInfoFields = 7;

    }  // namespace

    DataNode::DataNode() : Node("data_node")
    {
        // ── 参数 ──
        record_path_   = declare_parameter<std::string>("record_path", "");
        playback_path_ = declare_parameter<std::string>("playback_path", "");

        // ── Service: /data/command ──
        command_server_ = create_service<rus_sim_interfaces::srv::CommandService>(
            "/data/command",
            std::bind(&DataNode::handle_command, this,
                      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

        // ── 订阅 /driver/state（录制数据源） ──
        state_sub_ = create_subscription<rus_sim_interfaces::msg::RobotState>(
            "/driver/state", 10,
            std::bind(&DataNode::on_driver_state, this, std::placeholders::_1));

        // ── 发布回放状态 ──
        playback_state_pub_ = create_publisher<rus_sim_interfaces::msg::RobotState>("/data/state", 10);
        joint_state_pub_    = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

        // ── 回放定时器 ──
        playback_timer_ = create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / kPlaybackRate)),
            std::bind(&DataNode::playback_tick, this));

        // ── WebSocket 广播（回放状态 → 前端仿真界面） ──
        ws_server_.Start(kWsPort,
            [this](const std::string& cmd,
                   const std::vector<double>& args,
                   std::vector<double>& result) {
                return dispatch(cmd, args, result);
            },
            [this](int level, const std::string& msg) {
                switch (level) {
                    case 0:  RCLCPP_INFO(get_logger(), "%s", msg.c_str()); break;
                    case 1:  RCLCPP_WARN(get_logger(), "%s", msg.c_str()); break;
                    case 2:  RCLCPP_ERROR(get_logger(), "%s", msg.c_str()); break;
                }
            });

        last_tick_ = std::chrono::steady_clock::now();
        RCLCPP_INFO(get_logger(), "DataNode 已启动 ws://localhost:%d", kWsPort);
    }

    // ============================================================
    //  handle_command — /data/command 服务回调
    // ============================================================

    void DataNode::handle_command(
        const std::shared_ptr<rmw_request_id_t>,
        const std::shared_ptr<rus_sim_interfaces::srv::CommandService::Request> req,
        std::shared_ptr<rus_sim_interfaces::srv::CommandService::Response> res)
    {
        std::vector<double> result;
        res->success = dispatch(req->command, req->args, result);
        res->result = std::move(result);
        if (!res->success)
            res->message = "data command failed: " + req->command;
    }

    // ============================================================
    //  dispatch — 指令分发（ParseDataCommand + std::visit 静态多态）
    // ============================================================

    bool DataNode::dispatch(const std::string& cmd,
                            const std::vector<double>& args,
                            std::vector<double>& result)
    {
        auto command = ParseDataCommand(cmd, args);

        bool ok = std::visit(Overloaded{
            [&](const RecordStartCmd&) -> bool {
                if (recording_.load()) {
                    RCLCPP_WARN(get_logger(), "已在录制中");
                    return true;
                }
                if (!recorder_.StartRecording(record_path_)) {
                    RCLCPP_ERROR(get_logger(), "录制文件打开失败: %s", record_path_.c_str());
                    return false;
                }
                recording_.store(true);
                RCLCPP_INFO(get_logger(), "开始录制 → %s", record_path_.c_str());
                return true;
            },

            [&](const RecordStopCmd&) -> bool {
                if (!recording_.load()) {
                    RCLCPP_WARN(get_logger(), "当前未在录制");
                    return true;
                }
                recorder_.StopRecording();
                recording_.store(false);
                RCLCPP_INFO(get_logger(), "录制已停止（%zu 帧）", recorder_.GetFrameCount());
                return true;
            },

            [&](const PlaybackStartCmd&) -> bool {
                std::lock_guard<std::mutex> lock(playback_mutex_);
                if (!recorder_.LoadRecording(playback_path_)) {
                    RCLCPP_ERROR(get_logger(), "加载录制文件失败: %s", playback_path_.c_str());
                    return false;
                }
                playback_frames_.clear();
                playback_frames_.reserve(recorder_.GetFrameCount());
                for (size_t i = 0; i < recorder_.GetFrameCount(); ++i) {
                    RusUtils::RobotState st;
                    recorder_.GetFrame(i, st);
                    playback_frames_.push_back(std::move(st));
                }
                playback_ctrl_.SetFrames(&playback_frames_);
                playback_ctrl_.Play();
                playback_active_.store(true);
                last_tick_ = std::chrono::steady_clock::now();
                RCLCPP_INFO(get_logger(), "开始回放, %zu 帧", playback_frames_.size());
                return true;
            },

            [&](const PlaybackStopCmd&) -> bool {
                std::lock_guard<std::mutex> lock(playback_mutex_);
                playback_ctrl_.Stop();
                playback_active_.store(false);
                RCLCPP_INFO(get_logger(), "回放已停止");
                return true;
            },

            [&](const PlaybackPauseCmd&) -> bool {
                std::lock_guard<std::mutex> lock(playback_mutex_);
                playback_ctrl_.Pause();
                return true;
            },

            [&](const PlaybackResumeCmd&) -> bool {
                std::lock_guard<std::mutex> lock(playback_mutex_);
                playback_ctrl_.Play();
                return true;
            },

            [&](const PlaybackSetSpeedCmd& c) -> bool {
                std::lock_guard<std::mutex> lock(playback_mutex_);
                playback_ctrl_.SetSpeed(c.speed);
                return true;
            },

            [&](const PlaybackSeekCmd& c) -> bool {
                std::lock_guard<std::mutex> lock(playback_mutex_);
                return playback_ctrl_.SeekToTime(c.time_seconds);
            },

            [&](const PlaybackStepCmd& c) -> bool {
                std::lock_guard<std::mutex> lock(playback_mutex_);
                if (c.direction > 0) return playback_ctrl_.StepForward();
                return playback_ctrl_.StepBackward();
            },

            [&](const PlaybackSetLoopCmd& c) -> bool {
                std::lock_guard<std::mutex> lock(playback_mutex_);
                playback_ctrl_.SetLoop(c.enable);
                return true;
            },

            [&](const PlaybackGetInfoCmd&) -> bool {
                std::lock_guard<std::mutex> lock(playback_mutex_);
                result.reserve(kPlaybackInfoFields);
                result.push_back(static_cast<double>(playback_ctrl_.GetCurrentFrame()));
                result.push_back(static_cast<double>(playback_ctrl_.GetTotalFrames()));
                result.push_back(playback_ctrl_.GetCurrentTime());
                result.push_back(playback_ctrl_.GetTotalTime());
                result.push_back(playback_ctrl_.GetSpeed());
                result.push_back(playback_ctrl_.GetProgress());
                result.push_back(playback_active_.load() ? 1.0 : 0.0);
                return true;
            },

            [&](const auto&) -> bool {
                RCLCPP_WARN(get_logger(), "未知数据指令: %s", cmd.c_str());
                return false;
            },
        }, command);

        return ok;
    }

    // ============================================================
    //  on_driver_state — 录制数据源
    // ============================================================

    void DataNode::on_driver_state(const rus_sim_interfaces::msg::RobotState::SharedPtr msg)
    {
        if (!recording_.load()) return;

        RusUtils::RobotState st;
        auto to_eig = [](const std::vector<double>& v) {
            Eigen::VectorXd e(v.size());
            for (size_t i = 0; i < v.size(); ++i) e(i) = v[i];
            return e;
        };
        st.joint_pos   = to_eig(msg->joint_pos);
        st.joint_vel   = to_eig(msg->joint_vel);
        st.joint_acc   = to_eig(msg->joint_acc);
        st.effort      = to_eig(msg->effort);
        st.flange_pos  = to_eig(msg->flange_pos);
        st.timestamp   = msg->timestamp;
        recorder_.RecordFrame(st);
    }

    // ============================================================
    //  playback_tick — 回放推进与发布
    // ============================================================

    void DataNode::playback_tick()
    {
        if (!playback_active_.load()) {
            last_tick_ = std::chrono::steady_clock::now();
            return;
        }

        auto now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(now - last_tick_).count();
        last_tick_ = now;
        if (dt <= 0.0 || dt > 0.5) dt = 1.0 / kPlaybackRate;  // 限幅防跳跃

        RusUtils::RobotState out;
        {
            std::lock_guard<std::mutex> lock(playback_mutex_);
            bool ok = playback_ctrl_.Update(dt, out);
            if (!ok || !playback_ctrl_.IsPlaying()) {
                playback_active_.store(false);
                return;
            }
        }

        auto ros_state = to_ros_state(out);
        playback_state_pub_->publish(ros_state);

        // 同步发布 /joint_states（RViz / 前端使用）
        auto js = std::make_unique<sensor_msgs::msg::JointState>();
        js->header.stamp = rclcpp::Clock().now();
        js->name.resize(ros_state.joint_pos.size());
        for (size_t i = 0; i < js->name.size(); ++i)
            js->name[i] = "j" + std::to_string(i + 1);
        js->position = ros_state.joint_pos;
        js->velocity = ros_state.joint_vel;
        js->effort   = ros_state.effort;
        joint_state_pub_->publish(std::move(js));

        // WS 广播（前端仿真界面直接更新运动数据）
        ws_server_.Broadcast(state_to_json(out));
    }

    // ============================================================
    //  工具
    // ============================================================

    rus_sim_interfaces::msg::RobotState DataNode::to_ros_state(const RusUtils::RobotState& s)
    {
        rus_sim_interfaces::msg::RobotState msg;
        auto to_vec = [](const auto& eig) -> std::vector<double> {
            std::vector<double> v(eig.size());
            for (int i = 0; i < eig.size(); ++i) v[i] = eig(i);
            return v;
        };
        msg.joint_pos   = to_vec(s.joint_pos);
        msg.joint_vel   = to_vec(s.joint_vel);
        msg.joint_acc   = to_vec(s.joint_acc);
        msg.effort      = to_vec(s.effort);
        msg.flange_pos  = to_vec(s.flange_pos);
        msg.timestamp   = s.timestamp;
        return msg;
    }

    std::string DataNode::state_to_json(const RusUtils::RobotState& state)
    {
        auto to_json_arr = [](const auto& vec) -> std::string {
            std::string s;
            for (int i = 0; i < vec.size(); ++i) {
                if (i) s += ",";
                s += std::to_string(vec(i));
            }
            return "[" + s + "]";
        };

        return R"({"timestamp":)" + std::to_string(state.timestamp) +
               R"(,"frame_rate":)" + std::to_string(kPlaybackRate) +
               R"(,"joint_pos":)" + to_json_arr(state.joint_pos) +
               R"(,"joint_vel":)" + to_json_arr(state.joint_vel) +
               R"(,"joint_acc":)" + to_json_arr(state.joint_acc) +
               R"(,"effort":)" + to_json_arr(state.effort) +
               R"(,"flange_pos":)" + to_json_arr(state.flange_pos) + "}";
    }

}  // namespace RusSimData
