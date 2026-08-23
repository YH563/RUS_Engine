#include "rus_sim_driver/driver_node.hpp"
#include "driver/real_driver.hpp"

namespace RusDriverNode {

    DriverNode::DriverNode() : Node("driver_node") {
        // ── 参数 ──
        std::string driver_type = declare_parameter<std::string>("driver_type", "sim");
        std::string robot_ip   = declare_parameter<std::string>("robot_ip", "");
        script_path_           = declare_parameter<std::string>("script_path", "");

        // ── 创建驱动 ──
        robot_ip_ = robot_ip;
        if (driver_type == "sim") {
            driver_ = RusRobotDriver::DriverFactory::Create(
                RusRobotDriver::DriverFactory::Sim, robot_ip);
            is_sim_ = true;
            RCLCPP_INFO(get_logger(), "创建仿真驱动");
        } else {
            driver_ = RusRobotDriver::DriverFactory::Create(
                RusRobotDriver::DriverFactory::Real, robot_ip);
            is_sim_ = false;
            RCLCPP_INFO(get_logger(), "创建真实驱动, ip=%s", robot_ip.c_str());
            if (!driver_ || !driver_->IsConnected()) {
                // 读取真实驱动最近的 RPC 错误码，帮助定位连接失败原因
                int rpc_err = 0;
                if (driver_) {
                    if (auto* real = dynamic_cast<RusRealRobotDriver::RobotRealDriver*>(driver_.get()))
                        rpc_err = real->LastRpcError();
                }
                RCLCPP_ERROR(get_logger(),
                    "真实驱动连接失败 ip=%s rpc_err=%d（若错误码为 -2/-3 且 SDK 日志报 'error SDK version'，"
                    "说明 SDK 与控制器固件版本不匹配，请更换与控制器固件匹配的 SDK 版本）",
                    robot_ip.c_str(), rpc_err);
            }
        }

        // ── Service: /driver/command ──
        command_server_ = create_service<rus_sim_interfaces::srv::CommandService>(
            "/driver/command",
            std::bind(&DriverNode::handle_command, this,
                    std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

        // ── Publisher: /driver/state ──
        state_pub_ = create_publisher<rus_sim_interfaces::msg::RobotState>("/driver/state", 10);

        // ── Publisher: /joint_states ──
        joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

        // ── 关节名称 ──
        joint_names_ = {"j1", "j2", "j3", "j4", "j5", "j6"};

        // ── Timer: 125Hz 发布状态 ──
        timer_ = create_wall_timer(
            std::chrono::milliseconds(8),
            std::bind(&DriverNode::publish_state, this));

        RCLCPP_INFO(get_logger(), "DriverNode 启动完成");
    }

    RusSimRobotDriver::RobotSimDriver& DriverNode::sim_driver() {
        return static_cast<RusSimRobotDriver::RobotSimDriver&>(*driver_);
    }

    DriverNode::~DriverNode() = default;

    // ============================================================
    //  handle_command — /driver/command 服务回调
    // ============================================================
    void DriverNode::handle_command(
        const std::shared_ptr<rmw_request_id_t> req_header,
        const std::shared_ptr<rus_sim_interfaces::srv::CommandService::Request> req,
        std::shared_ptr<rus_sim_interfaces::srv::CommandService::Response> res)
    {
        (void)req_header;
        RCLCPP_INFO(get_logger(), "收到指令: %s (参数数: %zu)", req->command.c_str(), req->args.size());
        std::vector<double> result;
        res->success = dispatch(req->command, req->args, result);
        res->result = result;
        res->message = res->success ? "ok" : "unknown command";
        if (!res->success) {
            RCLCPP_WARN(get_logger(), "指令失败: %s", req->command.c_str());
        }
    }

    // ============================================================
    //  publish_state — 定时器：发布 /driver/state
    // ============================================================
    void DriverNode::publish_state() {
        if (!driver_) return;

        RusRobotDriver::RobotState state;
        if (driver_->GetCurrentState(1, state) != 0) return;

        // ── 转换为 ROS2 msg ──
        auto msg = std::make_unique<rus_sim_interfaces::msg::RobotState>();
        auto to_vec = [](const auto& eig) -> std::vector<double> {
            return {eig.data(), eig.data() + eig.size()};
        };
        msg->joint_pos  = to_vec(state.joint_pos);
        msg->joint_vel  = to_vec(state.joint_vel);
        msg->joint_acc  = to_vec(state.joint_acc);
        msg->effort     = to_vec(state.effort);
        msg->flange_pos = to_vec(state.flange_pos);
        msg->header.stamp = now();          // 标准时间戳（与 /joint_states 一致）
        msg->header.frame_id = "base_link";

        state_pub_->publish(std::move(msg));

        // ── 发布 /joint_states（用于 RViz） ──
        auto js = std::make_unique<sensor_msgs::msg::JointState>();
        js->header.stamp = now();
        js->header.frame_id = "base_link";
        js->name  = joint_names_;
        js->position = to_vec(state.joint_pos);
        js->velocity = to_vec(state.joint_vel);
        js->effort   = to_vec(state.effort);
        joint_state_pub_->publish(std::move(js));
    }

    // ============================================================
    //  dispatch — 指令分发（使用 ParseCommand + std::visit）
    // ============================================================
    bool DriverNode::dispatch(const std::string& cmd,
                              const std::vector<double>& args,
                              std::vector<double>& result)
    {
        using namespace RusRobotDriver;

        RCLCPP_INFO(get_logger(), "dispatch: %s (args=%zu)", cmd.c_str(), args.size());

        // ── switch_driver：参数格式特殊（含 IP 字符串），提前处理 ──
        if (cmd == RusRobotDriver::Cmd::kSwitchDriver) {
            uint8_t t = args.empty() ? 0 : static_cast<uint8_t>(args[0]);
            std::string ip;
            if (args.size() >= 5) {
                ip = std::to_string(static_cast<int>(args[1])) + "."
                   + std::to_string(static_cast<int>(args[2])) + "."
                   + std::to_string(static_cast<int>(args[3])) + "."
                   + std::to_string(static_cast<int>(args[4]));
            } else {
                ip = robot_ip_;
            }
            return switch_driver_impl(t, ip);
        }

        auto command = ParseCommand(cmd, args);

        bool ok = std::visit(Overloaded{
            [&](const MotionCommand& mc) -> bool {
                // 拷贝一份（driver 接口需要非 const 引用）
                auto cmd = mc;
                switch (cmd.type) {
                    case MOTION_TYPE_JOINT:   return driver_->MoveJ(cmd) == 0;
                    case MOTION_TYPE_CART:    return driver_->MoveL(cmd) == 0;
                    case MOTION_TYPE_SERVOJ:  return driver_->ServoJ(cmd) == 0;
                    case MOTION_TYPE_SERVOC:  return driver_->ServoCart(cmd) == 0;
                    case MOTION_TYPE_JOG_0:
                    case MOTION_TYPE_JOG_1:
                    case MOTION_TYPE_JOG_2:   return driver_->StartJOG(cmd) == 0;
                    default:                  return false;
                }
            },

            // ── 驱动控制 ──
            [&](const ConnectCmd&)        { return driver_->Connect(robot_ip_) == 0; },
            [&](const DisconnectCmd&)     { return driver_->Disconnect() == 0; },
            [&](const IsConnectedCmd&)    { result = {driver_->IsConnected() ? 1.0 : 0.0}; return true; },
            [&](const IsInDragTeachCmd&)  {
                uint8_t st = 0;
                int ret = driver_->IsInDragTeach(st);
                result = {static_cast<double>(st)};
                return ret == 0;
            },
            [&](const RobotEnableCmd& e)  { return driver_->RobotEnable(e.state) == 0; },
            [&](const GetStateCmd& g)     {
                RobotState st;
                int ret = driver_->GetCurrentState(g.flag, st);
                if (ret == 0) {
                    result.reserve(1 + st.joint_pos.size());
                    result.push_back(st.timestamp);
                    for (int i = 0; i < st.joint_pos.size(); ++i)
                        result.push_back(st.joint_pos(i));
                }
                return ret == 0;
            },
            [&](const IsMotionDoneCmd&)  { result = {driver_->IsMotionDone() ? 1.0 : 0.0}; return true; },

            // ── 仿真控制（仅 Sim 驱动有效；真实驱动下返回失败，避免非法向下转型） ──
            [&](const SetTimeSpeedCmd& s)  {
                if (!is_sim_) return false;
                sim_driver().SetTimeSpeed(s.speed); return true;
            },
            [&](const GetTimeSpeedCmd&)    {
                if (!is_sim_) return false;
                result = {sim_driver().GetTimeSpeed()}; return true;
            },
            [&](const GetSimTimeCmd&)      {
                if (!is_sim_) return false;
                result = {sim_driver().GetSimTime()}; return true;
            },
            [&](const StepOnceCmd&)        {
                if (!is_sim_) return false;
                sim_driver().StepOnce(); return true;
            },
            [&](const GetFrameRateCmd&)     {
                if (is_sim_) {
                    result = {sim_driver().GetFrameRate()};
                } else {
                    result = {125.0};  // 真实驱动占位
                }
                return true;
            },

            // ── 伺服模式 ──
            [&](const ServoStartCmd&)     { return driver_->ServoMoveStart() == 0; },
            [&](const ServoEndCmd&)       { return driver_->ServoMoveEnd() == 0; },

            // ── 运动控制 ──
            [&](const StopCmd&)           { return driver_->StopMotion() == 0; },
            [&](const PauseCmd&)          { return driver_->PauseMotion() == 0; },
            [&](const ResumeCmd&)         { return driver_->ResumeMotion() == 0; },
            [&](const ResetCmd& r)        { return driver_->ResetMotion(r) == 0; },
            [&](const StopJOGDecelCmd&)   { return driver_->StopJOGDecel() == 0; },
            [&](const StopJOGImmediateCmd&) { return driver_->StopJOGImmediate() == 0; },

            // ── 文件执行 ──
            [&](const RunFileCmd&)        {
                if (script_path_.empty()) {
                    RCLCPP_ERROR(get_logger(), "run_file 需要设置 script_path 参数");
                    return false;
                }
                return run_script(script_path_);
            },

            [](const auto&) { return false; }  // 兜底
        }, command);

        if (ok) {
            RCLCPP_INFO(get_logger(), "  → 成功 (result=%zu)", result.size());
        } else {
            RCLCPP_WARN(get_logger(), "  → 失败");
        }
        return ok;
    }

    // ============================================================
    //  switch_driver_impl — 运行时切换驱动
    // ============================================================
    bool DriverNode::switch_driver_impl(uint8_t type, const std::string& ip) {
        using Factory = RusRobotDriver::DriverFactory;

        Factory::Type new_type;
        bool new_is_sim;
        const char* type_str;
        if (type == 0) {
            new_type  = Factory::Sim;
            new_is_sim = true;
            type_str  = "sim";
        } else {
            new_type  = Factory::Real;
            new_is_sim = false;
            type_str  = "real";
        }

        // 如果类型和 IP 都没变，跳过
        if (new_is_sim == is_sim_ && ip == robot_ip_) {
            RCLCPP_INFO(get_logger(), "驱动类型和 IP 均未变化，跳过切换");
            return true;
        }

        // 停止当前运动（如有）
        if (driver_) {
            driver_->StopMotion();
            driver_->Disconnect();
        }

        // 重置旧驱动，创建新驱动
        driver_.reset();
        driver_ = Factory::Create(new_type, ip);
        if (!driver_) {
            RCLCPP_ERROR(get_logger(), "创建 %s 驱动失败", type_str);
            return false;
        }

        is_sim_   = new_is_sim;
        robot_ip_ = ip;

        if (!new_is_sim && !driver_->IsConnected()) {
            // 读取真实驱动最近的 RPC 错误码，帮助定位连接失败原因
            int rpc_err = 0;
            if (auto* real = dynamic_cast<RusRealRobotDriver::RobotRealDriver*>(driver_.get()))
                rpc_err = real->LastRpcError();
            RCLCPP_ERROR(get_logger(),
                "真实驱动连接失败 ip=%s rpc_err=%d（若错误码为 -2/-3 且 SDK 日志报 'error SDK version'，"
                "说明 SDK 与控制器固件版本不匹配，请更换与控制器固件匹配的 SDK 版本）",
                ip.c_str(), rpc_err);
        }

        RCLCPP_INFO(get_logger(), "驱动已切换为 %s (ip=%s)", type_str, ip.c_str());
        return true;
    }

    // ============================================================
    //  run_script — 执行指令文件
    // ============================================================
    bool DriverNode::run_script(const std::string& path) {
        std::ifstream file(path);
        if (!file) {
            RCLCPP_ERROR(get_logger(), "无法打开指令文件: %s", path.c_str());
            return false;
        }

        std::string line;
        int line_num = 0;
        while (std::getline(file, line)) {
            ++line_num;
            // 跳过空行和注释
            if (line.empty() || line[0] == '#') continue;

            std::istringstream iss(line);
            std::string cmd_str;
            iss >> cmd_str;
            if (cmd_str.empty()) continue;

            // 解析参数
            std::vector<double> args;
            double val;
            while (iss >> val) args.push_back(val);

            std::vector<double> result;
            bool ok = dispatch(cmd_str, args, result);
            if (!ok) {
                RCLCPP_WARN(get_logger(), "第 %d 行指令失败: %s", line_num, line.c_str());
            }
        }
        RCLCPP_INFO(get_logger(), "指令文件执行完毕: %s (%d 行)", path.c_str(), line_num);
        return true;
    }

}  // namespace RusDriverNode
