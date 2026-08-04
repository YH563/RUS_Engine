#include "rus_sim_app/coordinator.hpp"

namespace RusSimApp {

    // ── 单例与构造 ──

    Coordinator::SharedPtr Coordinator::get_instance()
    {
        if (!rclcpp::ok())
            rclcpp::init(0, nullptr);
        static SharedPtr instance(new Coordinator());
        return instance;
    }

    Coordinator::Coordinator()
        : rclcpp::Node("coordinator")
    {
        control_pub_ = this->create_publisher<std_msgs::msg::String>("/app/control", 10);
        driver_cmd_client_ =
            this->create_client<rus_sim_interfaces::srv::DriverCommand>("/driver/command");
        // TODO: 创建 user_interface（app WebSocket 服务器）
    }

    // ── 业务控制（经话题下发给 planning / driver） ──

    bool Coordinator::ExecutePreScan(double timeout)
    {
        if (!publish_control(RusUtils::Cmd::kPreScanStart, {}))
            return false;

        // TODO: 阻塞等待 planning 预扫查完成（经 IsMotionDone 或 planning 回执）
        (void)timeout;
        prescan_done_.store(true);
        return true;
    }

    bool Coordinator::ExecuteScan(double timeout)
    {
        if (!publish_control(RusUtils::Cmd::kExecute, {}))
            return false;

        // TODO: 阻塞等待 planning 扫查完成
        (void)timeout;
        return true;
    }

    void Coordinator::StopScan()
    {
        publish_control(RusUtils::Cmd::kStop, {});
    }

    // ── 状态查询 ──

    bool Coordinator::IsPreScanDone() const { return prescan_done_.load(); }

    bool Coordinator::IsMotionDone()
    {
        // TODO: 调用 /driver/command (is_motion_done) 查询当前动作是否完成
        return true;
    }

    // ── 用户指令入口（解析 + 自动分发） ──

    bool Coordinator::HandleUserCommand(const std::string& name, const std::vector<double>& args)
    {
        RusUtils::HighLevelCommand cmd;
        try {
            cmd = RusUtils::ParseHighLevelCommand(name, args);
        } catch (const std::exception& e) {
            RCLCPP_WARN(get_logger(), "未知指令: %s (%s)", name.c_str(), e.what());
            return false;
        }

        return std::visit(RusUtils::Overloaded{
            [this](const RusUtils::PreScanStartCmd&) { return ExecutePreScan(kDefaultTimeout); },
            [this](const RusUtils::SetStartPoseCmd& c) {
                return publish_control(RusUtils::Cmd::kSetStartPose, c.pose); },
            [this](const RusUtils::SetEndPoseCmd& c) {
                return publish_control(RusUtils::Cmd::kSetEndPose, c.pose); },
            [this](const RusUtils::PlanCmd&) { return publish_control(RusUtils::Cmd::kPlan, {}); },
            [this](const RusUtils::ExecuteCmd&) { return ExecuteScan(kDefaultTimeout); },
            [this](const RusUtils::StopCmd&) { StopScan(); return true; },
            [this](const RusUtils::PauseCmd&) { return publish_control(RusUtils::Cmd::kPause, {}); },
            [this](const RusUtils::ResumeCmd&) { return publish_control(RusUtils::Cmd::kResume, {}); },
            [this](const RusUtils::ResetCmd&) { return publish_control(RusUtils::Cmd::kReset, {}); },
            [this](const RusUtils::ConnectCmd&) { return publish_control(RusUtils::Cmd::kConnect, {}); },
            [this](const RusUtils::ShutdownCmd&) { return publish_control(RusUtils::Cmd::kShutdown, {}); },
        }, cmd);
    }

    bool Coordinator::publish_control(std::string_view name, const std::vector<double>& args)
    {
        // 序列化：name,arg1,arg2,...（TODO: 后续可换自定义控制消息）
        std::string data(name);
        for (double a : args)
            data += "," + std::to_string(a);

        std_msgs::msg::String msg;
        msg.data = data;
        control_pub_->publish(msg);
        return true;
    }

    // ── 前端交互（阻塞等待） ──

    std::string Coordinator::WaitForUserCommand(double timeout)
    {
        // TODO: 等待前端业务指令（start_prescan / start_scan / shutdown）
        (void)timeout;
        return "timeout";
    }

}  // namespace RusSimApp
